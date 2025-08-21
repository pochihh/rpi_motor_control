// enc_test_gpiod.cpp
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
#include <thread>
#include <vector>

struct EncCfg { int a_line; int b_line; };

struct Encoder {
    EncCfg cfg;
    gpiod_line* a{nullptr};
    gpiod_line* b{nullptr};
    int a_fd{-1}, b_fd{-1};
    std::atomic<int32_t> count{0};
    std::atomic<uint32_t> illegal{0};
    std::atomic<uint8_t> state{0}; // (A<<1)|B
};

static std::atomic<bool> running{true};

// Quadrature transition table: old -> new => delta
static const int8_t qdelta[4][4] = {
/*old  new: 00  01  10  11 */
 /*00*/ { 0, +1, -1,  0},
 /*01*/ {-1,  0,  0, +1},
 /*10*/ {+1,  0,  0, -1},
 /*11*/ { 0, -1, +1,  0}
};

static void apply_edge_event(Encoder& e)
{
    int a = gpiod_line_get_value(e.a);
    int b = gpiod_line_get_value(e.b);
    if (a < 0 || b < 0) return;
    uint8_t new_state = ((a ? 1 : 0) << 1) | (b ? 1 : 0);
    uint8_t old_state = e.state.load(std::memory_order_relaxed);
    int8_t delta = qdelta[old_state][new_state];
    if (delta == 0 && new_state != old_state) {
        e.illegal.fetch_add(1, std::memory_order_relaxed);
    }
    if (delta) e.count.fetch_add(delta, std::memory_order_relaxed);
    e.state.store(new_state, std::memory_order_relaxed);
}

int main(int argc, char** argv)
{
    // Encoders: A/B line offsets on gpiochip0 (edit if your mapping differs)
    std::array<Encoder, 6> encs {{
        {{5, 6}},    // enc0: GPIO5, GPIO6
        {{12, 13}},  // enc1
        {{16, 17}},  // enc2
        {{18, 19}},  // enc3
        {{22, 23}},  // enc4
        {{24, 25}}   // enc5
    }};

    double COUNTS_PER_REV = 4096.0; // 4x * 1024 CPR by default
    for (int i=1; i<argc; ++i) {
        if (!std::strcmp(argv[i], "--cpr") && i+1<argc) {
            double cpr = std::atof(argv[++i]);
            if (cpr > 0) COUNTS_PER_REV = 4.0 * cpr;
        }
    }

    std::signal(SIGINT, [](int){ running.store(false); });

    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) { std::perror("gpiod_chip_open"); return 1; }

    // Request lines and events
    for (auto& e : encs) {
        e.a = gpiod_chip_get_line(chip, e.cfg.a_line);
        e.b = gpiod_chip_get_line(chip, e.cfg.b_line);
        if (!e.a || !e.b) { std::fprintf(stderr, "Failed to get lines\n"); return 1; }

        // Request both-edges events with internal pull-ups if available
        if (gpiod_line_request_both_edges_events_flags(
                e.a, "enc_a", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) < 0 ||
            gpiod_line_request_both_edges_events_flags(
                e.b, "enc_b", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) < 0) {
            std::perror("gpiod_line_request_both_edges_events");
            return 1;
        }

        // Initialize state
        int a = gpiod_line_get_value(e.a);
        int b = gpiod_line_get_value(e.b);
        if (a < 0 || b < 0) { std::fprintf(stderr, "read init failed\n"); return 1; }
        e.state.store(((a?1:0)<<1)|(b?1:0), std::memory_order_relaxed);

        // Event fds
        e.a_fd = gpiod_line_event_get_fd(e.a);
        e.b_fd = gpiod_line_event_get_fd(e.b);
    }

    std::puts("enc_test_gpiod running. Ctrl-C to stop.");
    std::puts("Columns: enc#, count, counts/s, rev/s, illegal+");

    std::array<int32_t, 6> last_counts{};
    std::array<uint32_t, 6> last_illegal{};
    auto t0 = std::chrono::steady_clock::now();
    auto last = t0;

    // Build a poll list of all A/B fds
    std::vector<pollfd> fds;
    fds.reserve(encs.size()*2);
    for (auto& e : encs) {
        fds.push_back({e.a_fd, POLLIN, 0});
        fds.push_back({e.b_fd, POLLIN, 0});
    }

    while (running.load()) {
        // Wait up to 100 ms for any edge
        int ret = poll(fds.data(), fds.size(), 100);
        if (ret > 0) {
            // For each fd with data, read & apply event
            for (size_t i=0; i<fds.size(); ++i) {
                if (fds[i].revents & POLLIN) {
                    gpiod_line_event ev;
                    Encoder& e = encs[i/2];
                    gpiod_line* ln = (i%2==0) ? e.a : e.b;
                    // Drain all pending events on this line
                    while (gpiod_line_event_read(ln, &ev) == 0) {
                        (void)ev; // we re-read levels for robustness
                        apply_edge_event(e);
                    }
                }
            }
        }

        // Periodic print every ~500 ms
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        if (dt >= 0.5) {
            double tsec = std::chrono::duration<double>(now - t0).count();
            last = now;

            for (size_t i=0; i<encs.size(); ++i) {
                int32_t c = encs[i].count.load(std::memory_order_relaxed);
                uint32_t ill = encs[i].illegal.load(std::memory_order_relaxed);
                int32_t dc = c - last_counts[i];
                uint32_t dill = ill - last_illegal[i];
                last_counts[i] = c;
                last_illegal[i] = ill;

                double cps = dc / dt;
                double rps = cps / COUNTS_PER_REV;

                std::printf("enc%zu: %10d  %9.1f cps  %9.6f rps  illegal+%u\n",
                            i, c, cps, rps, dill);
            }
            std::puts("----");
        }
    }

    for (auto& e : encs) {
        if (e.a) gpiod_line_release(e.a);
        if (e.b) gpiod_line_release(e.b);
    }
    gpiod_chip_close(chip);
    return 0;
}