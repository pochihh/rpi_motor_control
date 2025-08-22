// Motor.cpp
#include "Motor.h"
#include <algorithm>

Motor::Motor(const char *chipPath, int enc_a_line, int enc_b_line,
             Motoron &driver, uint8_t motorId, unsigned enc_debounce_us)
    : encoder_(chipPath, enc_a_line, enc_b_line, enc_debounce_us),
      driver_(driver),
      motorId_(motorId),
      pid_(),
      counts_per_rev_(4096.0),
      gear_(1.0),
      last_counts_(0),
      pos_rev_(0.0),
      ref_pos_(0.0),
      last_cmd_(0.0),
      enabled_(false),
      u_to_speed_(800.0)
{
  last_counts_ = encoder_.count();
}

void Motor::setCountsPerRev(double cpr4x) { counts_per_rev_ = cpr4x; }
void Motor::setGear(double gear) { gear_ = gear; }
void Motor::setPID(double kp, double ki, double kd) { pid_.setGains(kp, ki, kd); }
void Motor::enable(bool en)
{
  enabled_ = en;
  if (!en)
    driver_.coastAll();
}
bool Motor::isEnabled() const { return enabled_; }
void Motor::setReference(double rev) { ref_pos_ = rev; }
double Motor::position() const { return pos_rev_; }
double Motor::command() const { return last_cmd_; }
uint32_t Motor::encoderIllegal() const { return encoder_.illegal(); }

void Motor::update(double dt_s)
{
  const int32_t c = encoder_.count();
  const int32_t dc = c - last_counts_;
  last_counts_ = c;

  pos_rev_ += static_cast<double>(dc) / counts_per_rev_ / gear_;

  const double u = pid_.step(ref_pos_, pos_rev_, dt_s);

  int16_t speed = static_cast<int16_t>(
      std::max(-800.0, std::min(800.0, u * u_to_speed_)));
  last_cmd_ = speed;

  if (enabled_)
    driver_.setSpeed(motorId_, speed);
  else
    driver_.coastAll();
}