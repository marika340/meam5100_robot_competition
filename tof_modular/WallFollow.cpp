#include "WallFollow.h"

WallFollow::WallFollow(Drivetrain& dt, ToFArray& tof)
  : _dt(dt), _tof(tof),
    _pid(0.7f, 0.0f, 1.2f, 1000.0f)   // PD only; high I-clamp because Ki=0
{}

void WallFollow::onEnter() {
  _dt.stop();
  _pid.reset();
  _lastLoopMs = millis();
  _stallState   = StallState::OK;
  _lastLeftCmd  = 0;
  _lastRightCmd = 0;
  _lastFrontTofSample = _tof.front();
  _frontTofSampleMs   = millis();
  // Initial follow direction: whichever side is closer.
  // updateFollowDirection();
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
  _dt.driveDirect(0, 0);
  delay(150);

  // 2) Pivot in place until front is clear past turnClearDist
  while (_tof.front() < _turnClearDist) {
    if (_followRight) _dt.driveDirect(-_sharpTurnOffset,  _sharpTurnOffset);
    else              _dt.driveDirect( _sharpTurnOffset, -_sharpTurnOffset);
    _tof.update();
    delay(260); //CHANGED FROM 300, 200 AND 250 DO OK WITH CORNERS(NEED 3 FIXES) BUT STILL WANT TO AVOID COMPLETELY, 277 AVOIDED THE RAMP
    //288 WORKS BUT AVOID RAMP SOMETIMES
  }

  // 3) Brief stop before resuming wall-follow
  _dt.driveDirect(0, 0);
  delay(100);
  _pid.reset();

  Serial.printf("[WF] Corner cleared. L:%.0f F:%.0f R:%.0f\n",
                _tof.left(), _tof.front(), _tof.right());

  _cornerExitMs       = millis();  // start immunity window
  _lastFrontTofSample = _tof.front();  // reset front-stuck tracking
  _frontTofSampleMs   = millis();
}


// ── Front-ToF stuck detection ────────────────────────────────────────────
bool WallFollow::isFrontStuck() {
  // Skip if we just handled a corner (ramp immunity) or front is clear.
  if (millis() - _cornerExitMs < RAMP_IMMUNITY_MS) return false;
  float front = _tof.front();
  if (front > TOF_FRONT_STUCK_MAX) return false;

  // If the front reading has moved enough, reset the timer — not stuck.
  if (fabsf(front - _lastFrontTofSample) > TOF_FRONT_STUCK_THRESH) {
    _lastFrontTofSample = front;
    _frontTofSampleMs   = millis();
    return false;
  }

  return (millis() - _frontTofSampleMs) >= TOF_FRONT_STUCK_CONFIRM_MS;
}

// ── Stall detection ──────────────────────────────────────────────────────
bool WallFollow::isStalled() {
  bool cmdHighEnough = (abs(_lastLeftCmd)  > STALL_CMD_THRESH ||
                        abs(_lastRightCmd) > STALL_CMD_THRESH);
  unsigned long now = millis();
  float rpmL = fabsf(_dt.left().computeRPM(now));
  float rpmR = fabsf(_dt.right().computeRPM(now));
  bool rpmTooLow = (fabsf(rpmL) < STALL_RPM_THRESH &&
                   fabsf(rpmR) < STALL_RPM_THRESH);
  return cmdHighEnough && rpmTooLow;
}

// ── Stall recovery state machine ─────────────────────────────────────────
void WallFollow::handleStall() {
  unsigned long now = millis();
  switch (_stallState) {

    case StallState::OK:
      if (isStalled()) {
        _stallState   = StallState::DETECTING;
        _stallStartMs = now;
        Serial.println("[WF] Stall suspected — confirming...");
      } else if (isFrontStuck()) {
        Serial.println("[WF] Front ToF frozen — reversing!");
        _dt.driveDirect(-100, -100);
        _stallState     = StallState::REVERSING;
        _reverseStartMs = now;
      }
      break;

    case StallState::DETECTING:
      if (!isStalled()) {
        _stallState = StallState::OK;   // false alarm
      } else if (now - _stallStartMs >= STALL_CONFIRM_MS) {
        Serial.println("[WF] Stall confirmed — reversing!");
        _dt.driveDirect(-100, -100);
        _stallState     = StallState::REVERSING;
        _reverseStartMs = now;
      }
      break;

    case StallState::REVERSING:
      if (now - _reverseStartMs >= STALL_REVERSE_MS) {
        _dt.stop();
        delay(150);
        _pid.reset();
        _stallState         = StallState::OK;
        _lastFrontTofSample = _tof.front();  // reset front-stuck tracking
        _frontTofSampleMs   = millis();
        Serial.println("[WF] Stall recovery done — resuming.");
      }
      break;
  }
}

void WallFollow::update() {
  // Run stall recovery every loop tick (not gated by _loopPeriodMs).
  handleStall();
  if (_stallState == StallState::REVERSING) return;

  unsigned long now = millis();
  if (now - _lastLoopMs < _loopPeriodMs) return;
  float dt = (now - _lastLoopMs) / 1000.0f;
  _lastLoopMs = now;

  // Reassess which wall to follow when the front is unobstructed.
  // if (_tof.front() > _frontStopDist) {
  //   updateFollowDirection();
  // }

  // Hard obstacle ahead — pivot away.
  bool immuneToFront = (millis() - _cornerExitMs < RAMP_IMMUNITY_MS);
  if (!immuneToFront && _tof.front() < _frontStopDist) {
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
  _lastLeftCmd  = leftCmd;
  _lastRightCmd = rightCmd;
  _dt.driveDirect(leftCmd, rightCmd);

  Serial.printf("[WF] F:%.0f R:%.0f follow:%s err:%.1f ctrl:%.1f\n",
                _tof.front(), _tof.right(),
                _followRight ? "R" : "L", error, control);
}
