#include "Centering.h"

Centering::Centering(Drivetrain& dt, ToFArray& tofs)
  : _dt(dt), _tofs(tofs),
    _pid(0.7f, 0.0f, 1.2f, 1000.0f)
{
}

void Centering::onEnter() {
  Serial.println("Centering::onEnter");
  _pid.reset();
  _lastMs = millis();
  _done   = false;
  _frontStopThreshold = 0.0f;
  _dt.stop();    // clean slate; raw drive will take over from update()
}

void Centering::update() {
  if (_done) return;

  unsigned long now = millis();
  float dt = (now - _lastMs) / 1000.0f;
  if (dt < MIN_DT_SEC) return;
  _lastMs = now;

  // PD on (right - left): positive err means we're closer to the right
  // wall, so steer left. Compute call gets -err so a positive ctrl
  // means "steer right" the same way the original free function did.
  float err = _tofs.right() - _tofs.left();
  if (fabsf(err) < DEADBAND_MM) err = 0.0f;
  float ctrl = _pid.compute(0.0f, -err, dt);
  ctrl = constrain(ctrl, -CTRL_LIMIT, CTRL_LIMIT);

  _dt.driveDirect(_baseSpeed + (int)ctrl, _baseSpeed - (int)ctrl);

  // Optional: stop when the front sensor sees we've arrived at the wall/tower.
  if (_frontStopThreshold > 0.0f && _tofs.front() < _frontStopThreshold) {
    Serial.printf("Centering::done (front=%.1f < %.1f)\n",
                  _tofs.front(), _frontStopThreshold);
    _dt.stop();
    _done = true;
  }
}

void Centering::onExit() {
  Serial.println("Centering::onExit");
  _dt.stop();
}
