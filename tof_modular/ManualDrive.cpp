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
  _dirL = 0; _dirR = 0;
  _targetRPM = 0.0f;

  _active = true;
  Serial.println(">>> ManualDrive::onEnter");
}

void ManualDrive::update() {
  _dt.runPidTick(millis());
}

void ManualDrive::onExit() {
  _dt.stop();
  _active = false;
}

void ManualDrive::setTargetRPM(float rpm) {
  _targetRPM = rpm;
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
