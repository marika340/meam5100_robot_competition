#include "LowTower.h"

LowTower::LowTower(Drivetrain& dt,
                   Centering&  centering,
                   PressTower& presser,
                   int         straightInches,
                   float       frontStopMm,
                   int         rotateDir)
  : _dt(dt), _centering(centering), _presser(presser),
    _straightInches(straightInches),
    _frontStopMm(frontStopMm),
    _rotateDir(rotateDir)
{
}

void LowTower::onEnter() {
  Serial.println("LowTower::onEnter (sequence start)");
  _dt.stop();
  _step = LT_STRAIGHT;
  _dt.straightMove(_straightInches);
}

void LowTower::update() {
  unsigned long now = millis();

  switch (_step) {
    case LT_STRAIGHT:
      if (_dt.updateStraightMove(now)) {
        Serial.println("LowTower: straight done -> rotate 1");
        _step = LT_ROTATE_1;
        _dt.rotateNinety(_rotateDir);
      }
      break;

    case LT_ROTATE_1:
      if (_dt.updateRotateNinety(now)) {
        Serial.println("LowTower: rotate 1 done -> rotate 2");
        _step = LT_ROTATE_2;
        _dt.rotateNinety(_rotateDir);
      }
      break;

    case LT_ROTATE_2:
      if (_dt.updateRotateNinety(now)) {
        Serial.println("LowTower: rotate 2 done -> centering");
        _step = LT_CENTER;
        // Configure Centering to auto-stop when we reach the tower,
        // then activate it as a nested Mode.
        _centering.setFrontStopThreshold(_frontStopMm);
        _centering.onEnter();
      }
      break;

    case LT_CENTER:
      _centering.update();
      if (_centering.isDone()) {
        _centering.onExit();
        // Restore default behaviour for any later standalone activation
        // of Centering (open-ended) via /mode=2.
        _centering.setFrontStopThreshold(0.0f);
        Serial.println("LowTower: centering done -> press");
        _step = LT_PRESS;
        _presser.onEnter();
      }
      break;

    case LT_PRESS:
      _presser.update();
      if (_presser.isDone()) {
        _presser.onExit();
        Serial.println("LowTower: press done -> sequence complete");
        _step = LT_DONE;
        // Don't switch modes here — the supervisor sees isDone() and
        // routes us back to MANUAL_DRIVE on the next loop tick.
      }
      break;

    case LT_DONE:
      // Idle. Supervisor will transition us out via isDone().
      break;
  }
}

void LowTower::onExit() {
  Serial.println("LowTower::onExit");
  // Tear down whichever sub-mode might still be active so we don't
  // leak state if the user aborts mid-sequence.
  switch (_step) {
    case LT_CENTER:
      _centering.onExit();
      _centering.setFrontStopThreshold(0.0f);
      break;
    case LT_PRESS:
      _presser.onExit();
      break;
    default:
      break;
  }
  _dt.stop();
  _step = LT_DONE;
}
