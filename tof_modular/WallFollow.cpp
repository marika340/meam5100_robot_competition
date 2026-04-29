#include "WallFollow.h"

WallFollow::WallFollow(Drivetrain& dt, ToFArray& tof)
  : _dt(dt), _tof(tof),
    _pid(0.7f, 0.0f, 1.2f, 1000.0f)   // PD only; high I-clamp because Ki=0
{}

void WallFollow::onEnter() {
  _dt.stop();
  _pid.reset();
  _lastLoopMs = millis();
  // Initial follow direction: whichever side is closer.
  updateFollowDirection();
  Serial.println(">>> WallFollow::onEnter — engaged.");
}

void WallFollow::onExit() {
  _dt.stop();
}

void WallFollow::updateFollowDirection() {
  float L = _tof.left();
  float R = _tof.right();
  if      (R + _switchMargin < L) _followRight = true;
  else if (L + _switchMargin < R) _followRight = false;
}

void WallFollow::handleCorner() {
  // 1) Stop and settle
  _dt.drive(0, 0);
  delay(150);

  // 2) Pivot in place until front is clear past turnClearDist
  while (_tof.front() < _turnClearDist) {
    if (_followRight) _dt.drive(-_sharpTurnOffset,  _sharpTurnOffset);
    else              _dt.drive( _sharpTurnOffset, -_sharpTurnOffset);
    _tof.update();
    delay(30);
  }

  // 3) Brief stop before resuming wall-follow
  _dt.drive(0, 0);
  delay(100);
  _pid.reset();

  Serial.printf("[WF] Corner cleared. L:%.0f F:%.0f R:%.0f\n",
                _tof.left(), _tof.front(), _tof.right());
}

void WallFollow::update() {
  unsigned long now = millis();
  if (now - _lastLoopMs < _loopPeriodMs) return;
  float dt = (now - _lastLoopMs) / 1000.0f;
  _lastLoopMs = now;

  // Reassess which wall to follow when the front is unobstructed.
  if (_tof.front() > _frontStopDist) {
    updateFollowDirection();
  }

  // Hard obstacle ahead — pivot away.
  if (_tof.front() < _frontStopDist) {
    handleCorner();
    return;
  }

  // Wall-distance PD. When the wall is lost we substitute a constant
  // search-curve command but keep prev_error tracking the live error
  // (matches original sketch — gives smoother re-engagement when the
  // wall comes back into range).
  float error;
  float control;
  if (_followRight) {
    error = _desiredWallDist - _tof.right();
    if (_tof.right() > _wallLostDist) {
      control = -50.0f;
      _pid.seedPrevError(error);
    } else {
      control = _pid.compute(0.0f, -error, dt);
    }
  } else {
    error = _tof.left() - _desiredWallDist;
    if (_tof.left() > _wallLostDist) {
      control = 50.0f;
      _pid.seedPrevError(error);
    } else {
      control = _pid.compute(0.0f, -error, dt);
    }
  }
  control = constrain(control, -120.0f, 120.0f);

  int leftCmd  = constrain(_baseSpeed - (int)control, _minSpeed, _maxSpeed);
  int rightCmd = constrain(_baseSpeed + (int)control, _minSpeed, _maxSpeed);
  _dt.drive(leftCmd, rightCmd);

  Serial.printf("[WF] F:%.0f R:%.0f follow:%s err:%.1f ctrl:%.1f\n",
                _tof.front(), _tof.right(),
                _followRight ? "R" : "L", error, control);
}
