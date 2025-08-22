#include "util.h"
#include <thread>
#include <sched.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>

void set_realtime(int prio)
{
  struct sched_param sp{};
  sp.sched_priority = prio;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
  {
    throw std::runtime_error(std::string("sched_setscheduler: ") + std::strerror(errno));
  }
}

ThreadMonitor::ThreadMonitor(const char *name) : name_(name) {}

void ThreadMonitor::begin_iter()
{
  t_start_ = clock_t::now();
}

void ThreadMonitor::end_iter(std::chrono::nanoseconds period)
{
  auto t_end = clock_t::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start_);
  busy_ns_ += elapsed;
  ++iters_;

  // Deadline / sleep-until
  auto next_deadline = t_start_ + period;
  if (t_end > next_deadline)
  {
    ++misses_;
    auto over = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - next_deadline).count();
    if (over > worst_overrun_ns_)
      worst_overrun_ns_ = over;
  }
  else
  {
    // spin-wait to be precise and avoid sleep jitter
    while (clock_t::now() < next_deadline)
      std::this_thread::yield();
  }
}

void ThreadMonitor::snapshot_reset(double &util_percent, uint64_t &iters, uint64_t &misses, int64_t &worst_overrun_ns)
{
  // Compute utilization = busy / (iters*period) — period is unknown here, so caller should supply period externally if needed.
  // We approximate utilization% across the last second by dividing busy time by (now - window_start).
  // Simpler: caller records period*iters for denominator per second; here we just hand out raw counters.
  iters = iters_;
  misses = misses_;
  worst_overrun_ns = worst_overrun_ns_;
  // util_percent will be computed by caller (needs period)
  // reset
  iters_ = 0;
  misses_ = 0;
  worst_overrun_ns_ = 0;
  busy_ns_ = std::chrono::nanoseconds(0);
  util_percent = -1.0; // caller will fill
}

const char *ThreadMonitor::name() const { return name_; }