#include "PressTower.h"

PressTower::PressTower(Drivetrain& dt,
                       unsigned long approachMs,
                       unsigned long holdMs,
                       unsigned long retreatMs,
                       int approachPwm,
                       int holdPwm,
                       int retreatPwm)
  : _dt(dt),
    _approachMs(approachMs),
    _holdMs(holdMs),
    _retreatMs(retreatMs),
    _approachPwm(approachPwm),
    _holdPwm(holdPwm),
    _retreatPwm(retreatPwm)
{
}

void PressTower::onEnter() {
  Serial.printf("PressTower::onEnter (hold=%lu ms)\n", _holdMs);
  _dt.stop();
  _phase        = PT_APPROACH;
  _phaseStartMs = millis();
  _dt.driveDirect( _approachPwm,  _approachPwm);
}

void PressTower::update() {
  unsigned long elapsed = millis() - _phaseStartMs;

  switch (_phase) {
    case PT_APPROACH:
      if (elapsed >= _approachMs) {
        _phase        = PT_HOLD;
        _phaseStartMs = millis();
        _dt.driveDirect(_holdPwm, _holdPwm);
      }
      break;
    case PT_HOLD:
      if (elapsed >= _holdMs) {
        _phase        = PT_RETREAT;
        _phaseStartMs = millis();
        _dt.driveDirect(-_retreatPwm, -_retreatPwm);
      }
      break;
    case PT_RETREAT:
      if (elapsed >= _retreatMs) {
        _dt.stop();
        _phase = PT_DONE;
        Serial.println("PressTower::done");
      }
      break;
    case PT_DONE:
      // Idle until supervisor (or composite mode) transitions us out.
      break;
  }
}

void PressTower::onExit() {
  Serial.println("PressTower::onExit");
  _dt.stop();
  _phase = PT_DONE;
}
