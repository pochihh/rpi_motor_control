#include "Encoder.h"
#include <poll.h>
#include <stdexcept>
#include <cstring>
#include <cerrno>
#include <ctime>

namespace
{
  inline uint64_t to_us(const timespec &ts)
  {
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
  }
  inline int readAB(gpiod_line *a, gpiod_line *b)
  {
    int va = gpiod_line_get_value(a);
    int vb = gpiod_line_get_value(b);
    if (va < 0 || vb < 0)
      return -1;
    return ((va ? 1 : 0) << 1) | (vb ? 1 : 0);
  }
  const int8_t qdelta[4][4] = {
      {0, +1, -1, 0},
      {-1, 0, 0, +1},
      {+1, 0, 0, -1},
      {0, -1, +1, 0}};
}

Encoder::Encoder(const char *chipPath, int a_line, int b_line, unsigned debounce_us)
    : debounce_us_(debounce_us)
{
  chip_ = gpiod_chip_open(chipPath);
  if (!chip_)
    throw std::runtime_error("gpiod_chip_open failed");

  a_ = gpiod_chip_get_line(chip_, a_line);
  b_ = gpiod_chip_get_line(chip_, b_line);
  if (!a_ || !b_)
    throw std::runtime_error("gpiod_chip_get_line failed");

  if (gpiod_line_request_both_edges_events_flags(a_, "enc_a", 0) < 0 ||
      gpiod_line_request_both_edges_events_flags(b_, "enc_b", 0) < 0)
  {
    throw std::runtime_error(std::string("line_request_events: ") + std::strerror(errno));
  }

  int st = readAB(a_, b_);
  if (st < 0)
    throw std::runtime_error("initial AB read failed");
  state_.store((uint8_t)st);

  a_fd_ = gpiod_line_event_get_fd(a_);
  b_fd_ = gpiod_line_event_get_fd(b_);
  th_ = std::thread(&Encoder::worker_, this);
}

Encoder::~Encoder()
{
  running_.store(false);
  if (th_.joinable())
    th_.join();
  if (a_)
    gpiod_line_release(a_);
  if (b_)
    gpiod_line_release(b_);
  if (chip_)
    gpiod_chip_close(chip_);
}

int32_t Encoder::count() const { return count_.load(); }
uint32_t Encoder::illegal() const { return illegal_.load(); }
void Encoder::zero()
{
  count_.store(0);
  illegal_.store(0);
}

void Encoder::worker_()
{
  // debounce timestamps per line
  uint64_t last_a_us = 0, last_b_us = 0;
  struct pollfd fds[2];
  fds[0].fd = a_fd_;
  fds[0].events = POLLIN;
  fds[1].fd = b_fd_;
  fds[1].events = POLLIN;

  while (running_.load())
  {
    int ret = poll(fds, 2, 100); // 100 ms timeout to check running flag
    if (ret < 0)
    {
      if (errno == EINTR)
        continue;
      break; // unexpected
    }
    if (ret == 0)
      continue;

    for (int i = 0; i < 2; ++i)
    {
      if (!(fds[i].revents & POLLIN))
        continue;

      gpiod_line *ln = (i == 0) ? a_ : b_;
      gpiod_line_event ev;
      // read ONE event (bounded), rely on next poll to fetch more
      if (gpiod_line_event_read(ln, &ev) == 0)
      {
        // debounce
        uint64_t now_us = to_us(ev.ts);
        uint64_t &last_us = (i == 0) ? last_a_us : last_b_us;
        if (debounce_us_ && (now_us - last_us) < debounce_us_)
        {
          last_us = now_us;
          continue;
        }
        last_us = now_us;

        // robust: re-read both levels and apply quad table
        int ns = readAB(a_, b_);
        if (ns < 0)
          continue;
        uint8_t old = state_.load();
        uint8_t neu = (uint8_t)ns;
        int8_t d = qdelta[old][neu];
        if (d == 0 && neu != old)
        {
          illegal_.fetch_add(1);
        }
        else if (d)
        {
          count_.fetch_add(d);
        }
        state_.store(neu);
      }
    }
  }
}