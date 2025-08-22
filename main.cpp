#include "util.h"
#include "Motoron.h"
#include "Motor.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <csignal>
#include <cmath>
#include <vector>

// using clock_t = std::chrono::steady_clock;
static std::atomic<bool> running{true};

int main()
{
  std::signal(SIGINT, [](int)
              { running = false; });
  std::signal(SIGTERM, [](int)
              { running = false; });

  // --- Hardware init ---
  Motoron motoron_1("/dev/i2c-1", 0x21);
  motoron_1.initBasic();
  // Motoron motoron_2("/dev/i2c-1", 0x22);
  // motoron_2.initBasic();

  // Build three motors; pass encoder lines (A,B)
  Motor m1("/dev/gpiochip0", 5, 6, motoron_1, 1, 5);
  m1.setCountsPerRev(4096);
  m1.setGear(1.0);
  m1.setPID(10, 40, 0.1);
  m1.enable(true);

  Motor m2("/dev/gpiochip0", 12, 13, motoron_1, 2, 5);
  m2.setCountsPerRev(4096);
  m2.setGear(1.0);
  m2.setPID(10, 40, 0.1);
  m2.enable(true);

  Motor m3("/dev/gpiochip0", 16, 17, motoron_1, 3, 5);
  m3.setCountsPerRev(4096);
  m3.setGear(1.0);
  m3.setPID(10, 40, 0.1);
  m3.enable(true);

  // initial setpoints
  m1.setReference(0.0);
  m2.setReference(0.0);
  m3.setReference(0.0);

  // Periods
  const auto period_ctrl = std::chrono::microseconds(1000); // 1 kHz
  const auto period_kine = std::chrono::milliseconds(5);        // 200 Hz

  // Monitors
  ThreadMonitor ctrl_monitor("control");
  ThreadMonitor kine_monitor("kinematics");

  // --- 1 kHz control thread ---
  std::thread control([&]
                      {
    try { set_realtime(80); } catch(...) {}
    const double dt = 0.001;
    while (running.load()) {
      ctrl_monitor.begin_iter();

      // Update each motor (encoder is interrupt-driven internally)
      m1.update(dt);
      m2.update(dt);
      m3.update(dt);

      ctrl_monitor.end_iter(period_ctrl);
    }
    motoron_1.coastAll(); });

  // --- 200 Hz kinematics thread ---
  std::thread kine([&]
                   {
    try { set_realtime(60); } catch(...) {}
    double t = 0.0;
    while (running.load()) {
      kine_monitor.begin_iter();

      t += std::chrono::duration<double>(period_kine).count();
      m1.setReference(25.0 * std::sin(2.0*3.1415926535*0.1*t));

      kine_monitor.end_iter(period_kine);
    } });

  // --- housekeeping: once per second, print thread stats ---
  std::thread hk([&]
                 {
    using namespace std::chrono;
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      double util_c=-1; uint64_t it_c=0, miss_c=0; int64_t worst_c=0;
      double util_k=-1; uint64_t it_k=0, miss_k=0; int64_t worst_k=0;

      // Snapshot counters and compute utilization:
      // Util% ≈ (iters * period - idle_time) — we only tracked busy time internally,
      // so simpler: assume loop was continuous over N iters and compute util as:
      // util = (busy_time) / (N * period).
      // We didn't expose busy_time; to keep it simple, estimate util from worst overruns and assume typical workload small.
      // For better accuracy, ThreadMonitor could expose busy_ns; here we just report misses & worst overrun.
      ctrl_monitor.snapshot_reset(util_c, it_c, miss_c, worst_c);
      kine_monitor.snapshot_reset(util_k, it_k, miss_k, worst_k);

      // Compute rough utilization as (iters * expected_workshare). Since we didn't retain busy_ns here,
      // report utilization as "N/A" and focus on misses/overruns. If you want exact %, we can extend the monitor.
      auto ns_to_us = [](int64_t ns){ return (double)ns/1000.0; };
      std::printf("[Threads] control: iters=%llu, misses=%llu, worst_overrun=%.1fus | "
                  "kinematics: iters=%llu, misses=%llu, worst_overrun=%.1fus | "
                  "pos=[%.4f, %.4f, %.4f], enc_illegal=[%u,%u,%u]\n",
        (unsigned long long)it_c, (unsigned long long)miss_c, ns_to_us(worst_c),
        (unsigned long long)it_k, (unsigned long long)miss_k, ns_to_us(worst_k),
        m1.position(), m2.position(), m3.position(),
        m1.encoderIllegal(), m2.encoderIllegal(), m3.encoderIllegal());
      std::fflush(stdout);
    } });

  control.join();
  kine.join();
  hk.join();
  return 0;
}