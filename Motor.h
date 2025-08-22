// Motor.h
#pragma once
#include "PID.h"
#include "Encoder.h"
#include "Motoron.h"
#include <cstdint>

class Motor
{
public:
  // motorId: Motoron channel (1..3)
  // encoder lines are passed here; Motor constructs its Encoder
  Motor(const char *chipPath, int enc_a_line, int enc_b_line,
        Motoron &driver, uint8_t motorId,
        unsigned enc_debounce_us = 5);

  void setCountsPerRev(double cpr4x);
  void setGear(double gear);
  void setPID(double kp, double ki, double kd);
  void enable(bool en);
  bool isEnabled() const;

  void setReference(double rev);
  double position() const;
  double command() const;
  uint32_t encoderIllegal() const;

  // called at 1 kHz
  void update(double dt_s);

private:
  Encoder encoder_;
  Motoron &driver_;
  uint8_t motorId_;

  PID pid_;
  double counts_per_rev_;
  double gear_;

  int32_t last_counts_;
  double pos_rev_;
  double ref_pos_;
  double last_cmd_;
  bool enabled_;

  double u_to_speed_;
};