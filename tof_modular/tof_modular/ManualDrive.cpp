#include "ManualDrive.h"

ManualDrive::ManualDrive(Drivetrain& dt)
  : _dt(dt),
    _active(false),
    _targetRPM(0.0f),
    _dirL(0), _dirR(0),
    _kp(2.0f), _ki(1.5f), _kd(0.8f)
{}

// there is a concern that the slider the user sees doesn't reflect the fact it's hard set to 0. User needs to re-enter a speed to update _targetRPM
void ManualDrive::onEnter() {
  // Clean slate. resetClosedLoop wipes target/dir/PID/encoders.
  _dt.resetClosedLoop();
  pushGains();

  // Drive command state resets per design — user re-touches slider.
  _targetRPM = 0.0f;
  _dirL = 0; _dirR = 0;
  _inKickstart = true;
  _kickStartUntil = millis() + 300;

  _active = true;
  Serial.println(">>> ManualDrive::onEnter");
}

void ManualDrive::update() {
  if (_inKickstart) {
    if (millis() < _kickStartUntil) {
      float kickstartPower = (_targetRPM * 0.9f) / 200.0f; 
      
      // Ensure we don't go below a minimum power to actually move
      if (kickstartPower < 0.4f) kickstartPower = 0.4f; 

      _dt.setPower(kickstartPower * _dirL, kickstartPower * _dirR);
      return;
    }
    _inKickstart = false;
    _dt.setDirection(_dirL, _dirR);
    _dt.setTargetRPM(_targetRPM);
  }
  _dt.runPidTick(millis());
}

void ManualDrive::onExit() {
  _dt.stop();
  _active = false;
}

void ManualDrive::setTargetRPM(float rpm) {
  _targetRPM = rpm;
  if (_active && rpm > 0.0f) {
    _kickStartUntil = millis() + 300;
    _inKickstart = true;
    _dt.setDirection(_dirL, _dirR);  // make sure direction is set first
  }
  if (_active) _dt.setTargetRPM(rpm);
}

void ManualDrive::setDirection(int leftDir, int rightDir) {
  _dirL = constrain(leftDir,  -1, 1);
  _dirR = constrain(rightDir, -1, 1);
  if (_active) _dt.setDirection(_dirL, _dirR);
}

void ManualDrive::setKp(float v) {
  _kp = v;
  if (_active) pushGains();
}

void ManualDrive::setKi(float v) {
  _ki = v;
  if (_active) pushGains();
}

void ManualDrive::setKd(float v) {
  _kd = v;
  if (_active) pushGains();
}

void ManualDrive::pushGains() {
  _dt.setPIDGains(_kp, _ki, _kd);
}
