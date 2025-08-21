#include <gpiod.h>
#include <poll.h>
#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

struct EncCfg { int a_line; int b_line; };
struct LineState {
    gpiod_line* ln{nullptr};
    int fd{-1};
    // For debounce
    uint64_t last_us{0};
};

struct Encoder {
    EncCfg cfg;
    LineState a, b;
    std::atomic<int32_t> count{0};
    std::atomic<uint32_t> illegal{0};
    std::atomic<uint8_t> state{0}; // (A<<1)|B
};

static std::atomic<bool> running{true};

// Old->new quad state delta
static const int8_t qdelta[4][4] = {
 /*old\new: 00  01  10  11 */
 /*00*/ { 0, +1, -1,  0},
 /*01*/ {-1,  0,  0, +1},
 /*10*/ {+1,  0,  0, -1},
 /*11*/ { 0, -1, +1,  0}
};

static inline uint64_t to_us(const timespec &ts) {
    return (uint64_t)ts.tv_sec * 1000000ull + ts.tv_nsec / 1000ull;
}

static inline int read_ab(gpiod_line* a, gpiod_line* b) {
    int va = gpiod_line_get_value(a);
    int vb = gpiod_line_get_value(b);
    if (va < 0 || vb < 0) return -1;
    return ((va ? 1 : 0) << 1) | (vb ? 1 : 0);
}

static void apply_event(Encoder& e)
{
    int ns = read_ab(e.a.ln, e.b.ln);
    if (ns < 0) return;
    uint8_t new_state = (uint8_t)ns;
    uint8_t old_state = e.state.load(std::memory_order_relaxed);
    int8_t d = qdelta[old_state][new_state];
    if (d == 0 && new_state != old_state) e.illegal.fetch_add(1, std::memory_order_relaxed);
    if (d) e.count.fetch_add(d, std::memory_order_relaxed);
    e.state.store(new_state, std::memory_order_relaxed);
}

int main(int argc, char** argv)
{
    // === Config ===
    std::array<Encoder, 6> encs {{
        {{5,6}}, {{12,13}}, {{16,17}}, {{18,19}}, {{22,23}}, {{24,25}}
    }};
    double COUNTS_PER_REV = 4096.0;   // 4x * CPR (default CPR=1024)
    uint32_t debounce_us   = 5;       // ignore edges within 5 us on same line
    int max_reads_per_tick = 5000;    // safety cap per 100 ms poll cycle (across all lines)

    for (int i=1; i<argc; ++i) {
        if (!std::strcmp(argv[i], "--cpr") && i+1<argc) {
            double cpr = std::atof(argv[++i]);
            if (cpr > 0) COUNTS_PER_REV = 4.0 * cpr;
        } else if (!std::strcmp(argv[i], "--debounce_us") && i+1<argc) {
            debounce_us = (uint32_t)std::atoi(argv[++i]);
        }
    }

    auto stop = [](int){ running.store(false); };
    std::signal(SIGINT,  stop);
    std::signal(SIGTERM, stop);
    std::signal(SIGHUP,  stop);
    std::signal(SIGQUIT, stop);
    std::signal(SIGTSTP, stop); // Ctrl-Z -> exit cleanly

    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) { std::perror("gpiod_chip_open"); return 1; }

    // Request lines and events
    for (auto &e : encs) {
        e.a.ln = gpiod_chip_get_line(chip, e.cfg.a_line);
        e.b.ln = gpiod_chip_get_line(chip, e.cfg.b_line);
        if (!e.a.ln || !e.b.ln) { std::fprintf(stderr, "get_line failed\n"); return 1; }

        // No bias flags to avoid kernel/driver incompat; use external pull-ups if needed
        if (gpiod_line_request_both_edges_events_flags(e.a.ln, "enc_a", 0) < 0 ||
            gpiod_line_request_both_edges_events_flags(e.b.ln, "enc_b", 0) < 0) {
            std::perror("request_both_edges_events");
            return 1;
        }

        // Init state
        int st = read_ab(e.a.ln, e.b.ln);
        if (st < 0) { std::fprintf(stderr, "init read failed\n"); return 1; }
        e.state.store((uint8_t)st, std::memory_order_relaxed);

        e.a.fd = gpiod_line_event_get_fd(e.a.ln);
        e.b.fd = gpiod_line_event_get_fd(e.b.ln);
    }

    std::puts("Running (Ctrl-C to stop). Columns: enc#, count, cps, rps, illegal+, drops");
    std::array<int32_t, 6> last_counts{};
    std::array<uint32_t, 6> last_illegal{};
    auto t0 = std::chrono::steady_clock::now();
    auto last = t0;

    // Build poll fds (A then B for each encoder)
    std::vector<pollfd> fds;
    fds.reserve(encs.size()*2);
    for (auto &e : encs) { fds.push_back({e.a.fd, POLLIN, 0}); fds.push_back({e.b.fd, POLLIN, 0}); }

    uint64_t dropped = 0;

    while (running.load()) {
        int ret = poll(fds.data(), fds.size(), 100); // 100 ms tick
        if (ret < 0) { if (errno == EINTR) continue; std::perror("poll"); break; }

        int reads = 0;
        if (ret > 0) {
            // Iterate lines, read at most max_reads_per_tick events in total
            for (size_t i=0; i<fds.size() && reads < max_reads_per_tick; ++i) {
                if (!(fds[i].revents & POLLIN)) continue;

                Encoder &e = encs[i/2];
                LineState &ls = (i%2==0) ? e.a : e.b;

                // Read ONE event per ready line this cycle
                gpiod_line_event ev;
                if (gpiod_line_event_read(ls.ln, &ev) == 0) {
                    ++reads;
                    // Software debounce using kernel timestamp
                    uint64_t us = to_us(ev.ts);
                    if (debounce_us && (us - ls.last_us) < debounce_us) {
                        ls.last_us = us;
                        continue; // drop as bounce
                    }
                    ls.last_us = us;

                    // Apply with fresh level read for robustness
                    apply_event(e);
                }
            }

            // If many lines were ready, we may have left some events in queue.
            // That’s fine; we’ll catch them next tick. If this happens constantly,
            // increase max_reads_per_tick or reduce noise/debounce_us.
            // Count how many we skipped (approx) for visibility:
            if (reads >= max_reads_per_tick) dropped++;
        }

        // Periodic status every ~500 ms
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        if (dt >= 0.5) {
            last = now;
            for (size_t i=0; i<encs.size(); ++i) {
                int32_t c = encs[i].count.load(std::memory_order_relaxed);
                uint32_t ill = encs[i].illegal.load(std::memory_order_relaxed);
                int32_t dc = c - last_counts[i];
                uint32_t dill = ill - last_illegal[i];
                last_counts[i] = c; last_illegal[i] = ill;

                double cps = dc / dt;
                double rps = cps / COUNTS_PER_REV;

                std::printf("enc%zu: %10d  %9.1f cps  %9.6f rps  illegal+%u  drops=%llu\n",
                            i, c, cps, rps, dill, (unsigned long long)dropped);
            }
            std::puts("----");
            dropped = 0;
        }
    }

    for (auto &e : encs) { if (e.a.ln) gpiod_line_release(e.a.ln); if (e.b.ln) gpiod_line_release(e.b.ln); }
    gpiod_chip_close(chip);
    return 0;
}