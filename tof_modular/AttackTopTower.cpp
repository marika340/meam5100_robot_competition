#include "AttackTopTower.h"

AttackTopTower::AttackTopTower(Drivetrain&    dt,
                               WallFollow&    wallFollow,
                               RobotPosition& robotPos,
                               ToFArray&      tofs,
                               PressTower&    presser,
                               Centering&     centering)
  : _dt(dt), _wf(wallFollow), _pos(robotPos), _tofs(tofs), _presser(presser), _centering(centering)
{
}

void AttackTopTower::onEnter() {
  Serial.println("AttackTopTower::onEnter -- engaging wall-follow to bridge");
  _dt.stop();

  // Reset all gating state for a clean run.
  _entryHits   = 0;
  _trigHits    = 0;
  _exitHits    = 0;
  _entryFlag   = false;
  _lastSampleMs = millis();
  // Capture baseline for the button bump detection
  _thicknessBaseline = _tofs.left();
  _rampStartMs = millis();

  _step = ATT_WALL_TO_BRIDGE;
  _wf.onEnter();
}

// ── Vive gating ──────────────────────────────────────────────────────────
//
// Counts CONSECUTIVE in-window samples at a fixed cadence (so we don't
// count one Vive frame multiple times). A miss resets the relevant
// counter back to zero — that's "10 in a row" 
void AttackTopTower::updateViveCounters() {
  unsigned long now = millis();
  if (now - _lastSampleMs < _samplePeriodMs) return;
  _lastSampleMs = now;

  RobotPosition::Position p = _pos.getRobotPosition(MID);
  // Vive 0 means "no fix this frame" (see RobotPosition::updateTracker
  // when status != VIVE_RECEIVING). Treat as a miss for every counter.
  if (p.x <= 0.5f && p.y <= 0.5f) {
    _entryHits = 0;
    _trigHits  = 0;
    _exitHits  = 0;
    return;
  }

  // Entry gateway — only relevant before the entry flag fires.
  if (!_entryFlag) {
    if (inWindow(p.x, p.y, _entryX, _entryY, _entryXTol, _entryYTol)) {
      if (_entryHits < 255) _entryHits++;
      if (_entryHits >= _confirmN) {
        _entryFlag = true;
        Serial.printf("[ATT] entry gateway confirmed at (%.0f,%.0f) -- now watching trigger\n",
                      p.x, p.y);
      }
    } else {
      _entryHits = 0;
    }
  }

  // Trigger window — only counted once we've cleared the entry gateway.
  if (_entryFlag) {
    if (inWindow(p.x, p.y, _trigX, _trigY, _trigXTol, _trigYTol)) {
      if (_trigHits < 255) _trigHits++;
    } else {
      _trigHits = 0;
    }

    // Exit/abort gateway — also only counted after the entry flag, so
    // jitter near (3622, 3000) before we even reached the bridge cannot
    // abort us prematurely.
    if (inWindow(p.x, p.y, _exitX, _exitY, _exitXTol, _exitYTol)) {
      if (_exitHits < 255) _exitHits++;
    } else {
      _exitHits = 0;
    }
  }
}

void AttackTopTower::enterRotate() {
  Serial.println("[ATT] trigger fired -- rotating 90 CCW");
  _wf.onExit();
  _dt.stop();
  _step = ATT_ROTATE_CCW;
  // Drivetrain::rotateNinety: dir==0 -> CW, anything else -> CCW.
  _dt.rotateNinety(/*CCW*/ 1);
}

void AttackTopTower::enterApproach() {
  Serial.println("[ATT] rotation done -- approaching button until front ToF stops us");
  _step = ATT_APPROACH;
  _dt.driveDirect(_approachPwm, _approachPwm);
}

void AttackTopTower::enterPress() {
  Serial.println("[ATT] front ToF threshold hit -- pressing");
  _dt.stop();
  _step = ATT_PRESS;
  _presser.onEnter();
}

void AttackTopTower::abortToDone(const char* why) {
  Serial.printf("[ATT] aborting: %s -- exit-gateway overshoot, returning to manual\n", why);
  _wf.onExit();
  _dt.stop();
  _step = ATT_DONE;
}

void AttackTopTower::update() {
  switch (_step) {

    case ATT_WALL_TO_BRIDGE: {
      //_wf.update();
      //updateViveCounters();
      // Once the entry gate trips, we move to ON_BRIDGE state. The
      // robot keeps wall-following without interruption.
      if (_isPureToF) {
        // ToF logic
        _centering.onEnter(); 
        _step = ATT_RAMP_CENTERING;
        break; 
      }
      _wf.update();
      updateViveCounters();
      if (_entryFlag) {
        _step = ATT_WALL_ON_BRIDGE;
      }
      break;
    }

    case ATT_WALL_ON_BRIDGE: {
      _wf.update();
      updateViveCounters();
      // Trigger fires first wins.
      if (_trigHits >= 2) {
        enterRotate();
      } else if (_exitHits >= _confirmN) {
        // Overshot the trigger window without firing. Abort.
        abortToDone("crossed exit gateway");
      }
      break;
    }

    //ADD CENTERING CASE
    case ATT_RAMP_CENTERING: {
        _centering.update(); // Keep us centered
    unsigned long rampElapsed = millis() - _rampStartMs;
    
    bool buttonDetected = (_tofs.left() < (_thicknessBaseline - THICKNESS_THRESHOLD));
    bool timedOut = (rampElapsed > 5555); //CHANGED FROM 6000 MS
    
    if (buttonDetected) {
        Serial.println("[ATT] button detected via ToF");
        _lastSampleMs = millis();
        _step = ATT_STEER_MANEUVER;
    } else if (timedOut) {
        Serial.println("[ATT] timed out -- assuming at top of ramp");
        _lastSampleMs = millis();
        _step = ATT_STEER_MANEUVER;
    }
    break;
    }

        //ADD MANUVER CASE
    case ATT_STEER_MANEUVER: {
        unsigned long elapsed = millis() - _lastSampleMs; //TIME WHEN THE CAR SEE THE BUTTON
        if (elapsed < 500) { 
        // Steer right: Left motor faster than right
        // This creates the diagonal move you want
        _dt.driveDirect(140, 80); 
      } else {
        _dt.stop();
        enterRotate(); 
      }
      break;
    }

    case ATT_ROTATE_CCW: {
      if (_dt.updateRotateNinety(millis())) {
        enterApproach();
      }
      break;
    }

    case ATT_APPROACH: {
      // Re-issue the command each tick in case anything else writes
      // motor PWM. Cheap insurance.
      _dt.driveDirect(_approachPwm, _approachPwm);
      if (_tofs.front() <= _frontStopMm && _tofs.front() > 1.0f) {
        // (>1.0 guard: ToF returns ~0 momentarily on bad reads; ignore.)
        enterPress();
      }
      break;
    }

    case ATT_PRESS: {
      _presser.update();
      if (_presser.isDone()) {
        _presser.onExit();
        Serial.println("[ATT] press done -- sequence complete");
        _step = ATT_DONE;
      }
      break;
    }

    case ATT_DONE:
      // Idle. Supervisor sees isDone() and routes us back to MANUAL_DRIVE.
      break;
  }
}

void AttackTopTower::onExit() {
  Serial.println("AttackTopTower::onExit");
  // Tear down whichever sub-mode might still be active so we don't
  // leak state if the user aborts mid-sequence.
  switch (_step) {
    case ATT_WALL_TO_BRIDGE:
    case ATT_WALL_ON_BRIDGE:
      _wf.onExit();
      break;
    case ATT_PRESS:
      _presser.onExit();
      break;
    default:
      break;
  }
  _dt.stop();
  _step = ATT_DONE;
}
