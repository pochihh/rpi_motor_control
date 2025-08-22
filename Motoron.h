#pragma once
#include <cstdint>
#include <string>

#define CMD_GET_FIRMWARE_VERSION 0x87
#define CMD_SET_PROTOCOL_OPTIONS 0x8B
#define CMD_READ_EEPROM 0x93
#define CMD_WRITE_EEPROM 0x95
#define CMD_REINITIALIZE 0x96
#define CMD_RESET 0x99
#define CMD_GET_VARIABLES 0x9A
#define CMD_SET_VARIABLE 0x9C
#define CMD_COAST_NOW 0xA5
#define CMD_CLEAR_MOTOR_FAULT 0xA6
#define CMD_CLEAR_LATCHED_STATUS_FLAGS 0xA9
#define CMD_SET_LATCHED_STATUS_FLAGS 0xAC
#define CMD_SET_BRAKING 0xB1
#define CMD_SET_BRAKING_NOW 0xB2
#define CMD_SET_SPEED 0xD1
#define CMD_SET_SPEED_NOW 0xD2
#define CMD_SET_BUFFERED_SPEED 0xD4
#define CMD_SET_ALL_SPEEDS 0xE1
#define CMD_SET_ALL_SPEEDS_NOW 0xE2
#define CMD_SET_ALL_BUFFERED_SPEEDS 0xE4
#define CMD_SET_ALL_SPEEDS_USING_BUFFERS 0xF0
#define CMD_SET_ALL_SPEEDS_NOW_USING_BUFFERS 0xF3
#define CMD_RESET_COMMAND_TIMEOUT 0xF5
#define CMD_MULTI_DEVICE_ERROR_CHECK 0xF9
#define CMD_MULTI_DEVICE_WRITE 0xFA

class Motoron
{
public:
  explicit Motoron(const std::string &i2cDev = "/dev/i2c-1", uint8_t addr = 0x10);
  ~Motoron();

  void initBasic();                            // disable CRC, clear reset flag; enables outputs
  void setSpeed(uint8_t motor, int16_t speed); // [-800..800]
  void coastAll();
  void enable(bool en);
  bool isEnabled() const;

private:
  int fd_;
  uint8_t address_;
  bool enabled_;

  void openBus(const std::string &dev);
  void writeBytes(const uint8_t *data, size_t n);
};