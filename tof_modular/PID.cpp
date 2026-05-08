#include "PID.h"

PID::PID(float kp, float ki, float kd, float integralLimit)
  : _kp(kp), _ki(ki), _kd(kd),
    _iLimit(integralLimit),
    _integral(0.0f), _prevError(0.0f)
{}

void PID::setGains(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
}

void PID::reset() {
  _integral  = 0.0f;
  _prevError = 0.0f;
  _firstCall = true;
}

float PID::compute(float setpoint, float measured, float dt) {
  if (dt <= 0.0f) dt = 0.001f;
  float error = setpoint - measured;
  _integral   = constrain(_integral + error * dt, -_iLimit, _iLimit);
  float deriv = (error - _prevError) / dt;
  _firstCall = false;
  _prevError  = error;
  return _kp * error + _ki * _integral + _kd * deriv;
}
