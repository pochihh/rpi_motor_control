#pragma once
#include <gpiod.h>
#include <atomic>
#include <cstdint>
#include <thread>
#include <string>

class Encoder
{
public:
  // chipPath: "/dev/gpiochip0"; a_line/b_line: line offsets (e.g., 5 and 6)
  Encoder(const char *chipPath, int a_line, int b_line, unsigned debounce_us = 5);
  ~Encoder();

  // Counts are 4× quadrature counts
  int32_t count() const;
  void zero();
  uint32_t illegal() const;

private:
  // worker thread waits on edge events and updates counts
  void worker_();

  gpiod_chip *chip_{nullptr};
  gpiod_line *a_{nullptr};
  gpiod_line *b_{nullptr};
  int a_fd_{-1}, b_fd_{-1};
  unsigned debounce_us_;
  std::atomic<bool> running_{true};
  std::thread th_;

  // state
  std::atomic<int32_t> count_{0};
  std::atomic<uint32_t> illegal_{0};
  std::atomic<uint8_t> state_{0}; // (A<<1)|B
};