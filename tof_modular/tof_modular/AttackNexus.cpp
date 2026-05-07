#include "AttackNexus.h"
#include "ToFArray.h"

AttackNexus::AttackNexus(Drivetrain& dt,
                         Centering&  centering,
                         PressTower& presser,
                         ToFArray&   tofs,
                         int         straightInches,
                         float       frontStopMm,
                         int         pressCount)
  : _dt(dt), _centering(centering), _presser(presser), _tofs(tofs),
    _straightInches(straightInches),
    _frontStopMm(frontStopMm),
    _pressCount(pressCount)
{
}

void AttackNexus::onEnter() {
  Serial.println("AttackNexus::onEnter (sequence start)");
  _dt.stop();
  _pressesDone = 0;
  _step = AN_STRAIGHT;
  _dt.straightMove(_straightInches);
}

void AttackNexus::update() {
  unsigned long now = millis();
  (void)now;  // currently no per-step timing here

  switch (_step) {
    case AN_STRAIGHT:
      if (_tofs.front() <= 1300 && _tofs.front() > 1.0f) {    //CHANGE 888 IF THE DISTANCE IS NOT ENOUGH FOR CENTERING
        Serial.println("AttackNexus: ToF early stop -> centering");
        _dt.stop();
        _step = AN_CENTER;
        _centering.setFrontStopThreshold(_frontStopMm);
        _centering.onEnter();
      } else if (_dt.updateStraightMove(now)) {
        Serial.println("AttackNexus: straight done -> centering");
        _step = AN_CENTER;
        _centering.setFrontStopThreshold(_frontStopMm);
        _centering.onEnter();
      }
      break;

    case AN_CENTER:
      _centering.update();
      if (_centering.isDone()) {
        _centering.onExit();
        // Restore default behaviour so a later standalone Centering
        // activation via /mode=2 stays open-ended.
        _centering.setFrontStopThreshold(0.0f);
        Serial.println("AttackNexus: centering done -> press 1");
        _step = AN_PRESS;
        _presser.onEnter();
      }
      break;

    case AN_PRESS:
      _presser.update();
      if (_presser.isDone()) {
        _presser.onExit();
        _pressesDone++;
        Serial.printf("AttackNexus: press %d/%d done\n",
                      _pressesDone, _pressCount);

        if (_pressesDone < _pressCount) {
          // Re-engage: PressTower's approach phase will drive forward
          // again to the next button, hold, then retreat. Sequencing
          // between buttons is delegated entirely to PressTower's
          // approach/hold/retreat phases.
          Serial.printf("AttackNexus: re-engage for press %d\n",
                        _pressesDone + 1);
          _presser.onEnter();
        } else {
          // Final press complete -- the last retreat already backed us
          // off, so just flag done. Supervisor will route us back to
          // MANUAL_DRIVE on the next loop tick.
          Serial.println("AttackNexus: all presses complete -> done");
          _dt.stop();
          _step = AN_DONE;
        }
      }
      break;

    case AN_DONE:
      // Idle. Supervisor will transition us out via isDone().
      break;
  }
}

void AttackNexus::onExit() {
  Serial.println("AttackNexus::onExit");
  // Tear down whichever sub-mode might still be active so we don't
  // leak state if the user aborts mid-sequence.
  switch (_step) {
    case AN_CENTER:
      _centering.onExit();
      _centering.setFrontStopThreshold(0.0f);
      break;
    case AN_PRESS:
      _presser.onExit();
      break;
    default:
      break;
  }
  _dt.stop();
  _step = AN_DONE;
}
