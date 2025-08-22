#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <algorithm>
#include <limits>

/**
 * @brief A PID controller with anti-windup and output saturation.
 *
 * Features:
 *  - Back-calculation anti-windup (set anti_windup_gain_ > 0).
 *  - Conditional integration fallback when anti_windup_gain_ == 0.
 *  - Output saturation and integrator clamping.
 *  - Returns the saturated control command.
 *
 * Typical usage:
 *   PidController pid;
 *   pid.setGains(1.0, 0.5, 0.05);
 *   pid.setOutputSaturationLimits(-12.0, 12.0);   // e.g., motor voltage
 *   pid.setAntiWindupGain(0.5);                   // enable back-calculation
 *
 *   double u = pid.step(reference, measurement, dt_seconds);
 */
class PID
{
public:
  // Constructors
  PID() = default;
  PID(double proportional_gain,
                double integral_gain,
                double derivative_gain,
                double output_min = -1.0,
                double output_max = 1.0,
                double anti_windup_gain = 0.0);

  // Core step
  double step(double reference_value,
              double measured_value,
              double time_step_seconds);

  // Reset internal states (integrator and previous error)
  void reset(double integrator_state = 0.0, double previous_error = 0.0);

  // --- Configuration setters ---
  void setGains(double proportional_gain,
                double integral_gain,
                double derivative_gain);
  void setOutputSaturationLimits(double output_min, double output_max);
  void setIntegralStateLimits(double integrator_min, double integrator_max);
  void setAntiWindupGain(double anti_windup_gain);

  // --- Getters (useful for diagnostics/telemetry) ---
  double kp() const { return p_gain_; }
  double ki() const { return i_gain_; }
  double kd() const { return d_gain_; }

  double outputMin() const { return output_min_; }
  double outputMax() const { return output_max_; }

  double integratorMin() const { return integrator_min_; }
  double integratorMax() const { return integrator_max_; }

  double antiWindupGain() const { return anti_windup_gain_; }

  double integratorState() const { return integrator_state_; }
  double previousError() const { return previous_error_; }

  double lastControlUnclamped() const { return last_control_unclamped_; }
  double lastControlSaturated() const { return last_control_saturated_; }

private:
  // Helper clamp
  static inline double clamp(double value, double low, double high)
  {
    return std::max(low, std::min(value, high));
  }

  // Tunable gains
  double p_gain_{0.0};
  double i_gain_{0.0};
  double d_gain_{0.0};

  // Output saturation limits (actuator capabilities)
  double output_min_{-1.0};
  double output_max_{1.0};

  // Integrator state limits (numeric safety)
  double integrator_min_{-1e6};
  double integrator_max_{1e6};

  // Anti-windup back-calculation gain (0 => use conditional integration)
  double anti_windup_gain_{0.0};

  // Internal states
  double integrator_state_{0.0};
  double previous_error_{0.0};

  // Last computed outputs (for logging / diagnostics)
  double last_control_unclamped_{0.0};
  double last_control_saturated_{0.0};
};

#endif // PID_CONTROLLER_H_