// Motoron.cpp
#include "Motoron.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <cstring>
#include <algorithm>

Motoron::Motoron(const std::string &dev, uint8_t addr)
    : fd_(-1), address_(addr), enabled_(false) { openBus(dev); }

Motoron::~Motoron()
{
  if (fd_ >= 0)
    ::close(fd_);
}

void Motoron::openBus(const std::string &dev)
{
  fd_ = ::open(dev.c_str(), O_RDWR);
  if (fd_ < 0)
    throw std::runtime_error(std::string("open ") + dev + ": " + std::strerror(errno));
  if (ioctl(fd_, I2C_SLAVE, address_) < 0)
    throw std::runtime_error(std::string("I2C_SLAVE: ") + std::strerror(errno));
}
void Motoron::writeBytes(const uint8_t *data, size_t n)
{
  if (::write(fd_, data, n) != (ssize_t)n)
    throw std::runtime_error(std::string("i2c write: ") + std::strerror(errno));
}
void Motoron::initBasic()
{
  const uint8_t disable_crc[] = {CMD_SET_PROTOCOL_OPTIONS, 0x04};
  writeBytes(disable_crc, sizeof(disable_crc));
  
  const uint8_t clear_reset[] = {CMD_CLEAR_LATCHED_STATUS_FLAGS, 0x00, 0x04};
  writeBytes(clear_reset, sizeof(clear_reset));
  enabled_ = true;
}
void Motoron::setSpeed(uint8_t motor, int16_t speed)
{
  if (!enabled_)
    return;
  speed = std::max<int16_t>(-800, std::min<int16_t>(800, speed));
  uint8_t cmd[4];
  cmd[0] = CMD_SET_SPEED_NOW;
  cmd[1] = motor & 0x7F;
  cmd[2] = speed & 0x7F;
  cmd[3] = (speed >> 7) & 0x7F;
  writeBytes(cmd, sizeof(cmd));
}

void Motoron::coastAll()
{
  uint8_t cmd[8];
  cmd[0] = 0xE2;
  for (int i = 1; i < 8; ++i)
    cmd[i] = 0;
  writeBytes(cmd, sizeof(cmd));
}
void Motoron::enable(bool en)
{
  enabled_ = en;
  if (!en)
    coastAll();
}
bool Motoron::isEnabled() const { return enabled_; }