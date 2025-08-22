#include "PID.h"
#include <cmath>

PID::PID(double proportional_gain,
         double integral_gain,
         double derivative_gain,
         double output_min,
         double output_max,
         double anti_windup_gain)
    : p_gain_(proportional_gain),
      i_gain_(integral_gain),
      d_gain_(derivative_gain),
      output_min_(output_min),
      output_max_(output_max),
      anti_windup_gain_(anti_windup_gain)
{
  // Ensure sensible ordering for limits
  if (output_min_ > output_max_)
  {
    std::swap(output_min_, output_max_);
  }
}

void PID::setGains(double proportional_gain,
                   double integral_gain,
                   double derivative_gain)
{
  p_gain_ = proportional_gain;
  i_gain_ = integral_gain;
  d_gain_ = derivative_gain;
}

void PID::setOutputSaturationLimits(double output_min,
                                    double output_max)
{
  output_min_ = output_min;
  output_max_ = output_max;
  if (output_min_ > output_max_)
  {
    std::swap(output_min_, output_max_);
  }
}

void PID::setIntegralStateLimits(double integrator_min,
                                 double integrator_max)
{
  integrator_min_ = integrator_min;
  integrator_max_ = integrator_max;
  if (integrator_min_ > integrator_max_)
  {
    std::swap(integrator_min_, integrator_max_);
  }
}

void PID::setAntiWindupGain(double anti_windup_gain)
{
  anti_windup_gain_ = anti_windup_gain;
}

void PID::reset(double integrator_state, double previous_error)
{
  integrator_state_ = clamp(integrator_state, integrator_min_, integrator_max_);
  previous_error_ = previous_error;
  last_control_unclamped_ = 0.0;
  last_control_saturated_ = 0.0;
}

double PID::step(double reference,
                 double measurement,
                 double ts)
{
  // Guard against non-positive dt
  if (ts <= 0.0)
  {
    ts = 1e-6;
  }

  // Compute error
  const double error = reference - measurement;

  // Proportional term
  const double p_term = p_gain_ * error;

  // Derivative term (simple backward difference on error)
  const double d_term =
      d_gain_ * (error - previous_error_) / ts;

  // Form unclamped control using current integrator state
  const double control_unclamped =
      p_term + integrator_state_ + d_term;

  // Saturate to actuator capability
  const double control_clamped =
      clamp(control_unclamped, output_min_, output_max_);

  // --- Anti-windup handling ---
  if (anti_windup_gain_ > 0.0)
  {
    // Back-calculation: drive integrator to reduce (unclamped - saturated)
    const double anti_windup_correction =
        anti_windup_gain_ * (control_clamped - control_unclamped);
    integrator_state_ +=
        (i_gain_ * error + anti_windup_correction) * ts;
  }
  else
  {
    // Conditional integration: pause integration when pushing further into saturation
    const bool pushing_high =
        (control_unclamped >= output_max_) && (error > 0.0);
    const bool pushing_low =
        (control_unclamped <= output_min_) && (error < 0.0);
    if (!(pushing_high || pushing_low))
    {
      integrator_state_ += i_gain_ * error * ts;
    }
  }

  // Clamp integrator for numeric safety
  integrator_state_ = clamp(integrator_state_, integrator_min_, integrator_max_);

  // Update stored values for next iteration and diagnostics
  previous_error_ = error;
  last_control_unclamped_ = control_unclamped;
  last_control_saturated_ = control_clamped;

  // Return the command you can send to the actuator
  return control_clamped;
}