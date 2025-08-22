#pragma once
#include <chrono>

void set_realtime(int prio = 80);

// Simple per-thread timing monitor: measure busy time, utilization, deadline misses
class ThreadMonitor
{
public:
    explicit ThreadMonitor(const char *name = "thread");
    void begin_iter();                              // call at loop start
    void end_iter(std::chrono::nanoseconds period); // call at loop end; records stats & handles deadline
    // Call from housekeeping every ~1s to get a snapshot and reset window
    void snapshot_reset(double &util_percent, uint64_t &iters, uint64_t &misses, int64_t &worst_overrun_ns);

    const char *name() const;

private:
    const char *name_;
    using clock_t = std::chrono::steady_clock;
    clock_t::time_point t_start_{};
    // accumulators for the current window
    uint64_t iters_{0};
    uint64_t misses_{0};
    int64_t worst_overrun_ns_{0};
    std::chrono::nanoseconds busy_ns_{0};
};