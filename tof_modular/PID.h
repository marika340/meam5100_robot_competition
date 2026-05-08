#ifndef PID_H
#define PID_H

#include <Arduino.h>

// =====================================================================
// PID: pure controller. Knows nothing about motors, encoders, or
// distance sensors — give it a setpoint, a measurement, and a dt and
// it returns a control output. The caller decides how to apply it.
//
// The same class is used for:
//   - per-motor RPM control inside Drivetrain
//   - heading/wall-distance control inside WallFollow
//   - centering control (future)
//
// Integral has a symmetric clamp (set via setIntegralLimit) to prevent
// windup. seedIntegral() lets bumpless transfer code preload the
// integrator so output matches the current PWM at handoff.
// =====================================================================
class PID {
public:
  PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
      float integralLimit = 50.0f);

  void  setGains(float kp, float ki, float kd);
  void  setIntegralLimit(float limit) { _iLimit = limit; }
  void  reset();

  // Pre-load the integrator (used for bumpless manual->PID handoff).
  void  seedIntegral(float value)     { _integral = value; }

  // Pre-load previous error (so the first derivative term isn't a step).
  void  seedPrevError(float value)    { _prevError = value; }

  // Compute control output. dt in seconds (must be > 0).
  float compute(float setpoint, float measured, float dt);

  float kp() const { return _kp; }
  float ki() const { return _ki; }
  float kd() const { return _kd; }
  float integral()  const { return _integral; }
  float prevError() const { return _prevError; }

private:
  float _kp, _ki, _kd;
  float _iLimit;
  float _integral;
  float _prevError;
  bool _firstCall = true;
};

#endif // PID_H
