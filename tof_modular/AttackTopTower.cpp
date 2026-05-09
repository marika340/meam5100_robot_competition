#include "AttackTopTower.h"

AttackTopTower::AttackTopTower(Drivetrain& dt,
                               LowTower&   lowTower,
                               Centering&  centering,
                               PressTower& nexusPresser,
                               PressTower& finalPresser,
                               WallFollow& wallFollow,
                               int         backUpInches,
                               float       frontStopMm,
                               int         pressCount,
                               unsigned long wallFollowMs)
  : _dt(dt), _lowTower(lowTower), _centering(centering),
    _nexusPresser(nexusPresser), _finalPresser(finalPresser),
    _wallFollow(wallFollow),
    _backUpInches(backUpInches),
    _frontStopMm(frontStopMm),
    _pressCount(pressCount),
    _wallFollowMs(wallFollowMs)
{}

void AttackTopTower::onEnter() {
  Serial.println("AttackTopTower::onEnter -- starting LowTower");
  _dt.stop();
  _pressesDone    = 0;
  _wallFollowStart = 0;
  _step           = ATT_LOW_TOWER;
  _lowTower.onEnter();
}

void AttackTopTower::update() {
  unsigned long now = millis();

  switch (_step) {

    // ── 1. LowTower ──────────────────────────────────────────────────
    case ATT_LOW_TOWER:
      _lowTower.update();
      if (_lowTower.isDone()) {
        _lowTower.onExit();
        Serial.println("[ATT] LowTower done -> back up");
        _dt.straightMove(-_backUpInches);
        _step = ATT_BACK_UP;
      }
      break;

    // ── 2. Back up ───────────────────────────────────────────────────
    case ATT_BACK_UP:
      if (_dt.updateStraightMove(now)) {
        Serial.println("[ATT] back up done -> rotate 180 (1/2)");
        _dt.rotateNinety(0 /*CW*/);
        _step = ATT_ROTATE_180_1;
      }
      break;

    // ── 3a. First 90° CW ─────────────────────────────────────────────
    case ATT_ROTATE_180_1:
      if (_dt.updateRotateNinety(now)) {
        Serial.println("[ATT] rotate 180 (1/2) done -> rotate 180 (2/2)");
        _dt.rotateNinety(0 /*CW*/);
        _step = ATT_ROTATE_180_2;
      }
      break;

    // ── 3b. Second 90° CW ────────────────────────────────────────────
    case ATT_ROTATE_180_2:
      if (_dt.updateRotateNinety(now)) {
        delay(75);
        Serial.println("[ATT] rotate 180 (2/2) done -> centering");
        _centering.onEnter();
        _centering.setFrontStopThreshold(_frontStopMm);
        _step = ATT_CENTER;
      }
      break;

    // ── 4. Center to nexus ───────────────────────────────────────────
    case ATT_CENTER:
      _centering.update();
      if (_centering.isDone()) {
        _centering.onExit();
        _centering.setFrontStopThreshold(0.0f);
        Serial.println("[ATT] centering done -> press 1");
        _pressesDone = 0;
        _nexusPresser.onEnter();
        _step = ATT_PRESS;
      }
      break;

    // ── 5. Press nexus x4 ────────────────────────────────────────────
    case ATT_PRESS:
      _nexusPresser.update();
      if (_nexusPresser.isDone()) {
        _nexusPresser.onExit();
        _pressesDone++;
        Serial.printf("[ATT] press %d/%d done\n", _pressesDone, _pressCount);
        if (_pressesDone < _pressCount) {
          _nexusPresser.onEnter();
        } else {
          Serial.println("[ATT] all presses done -> rotate CW");
          _dt.rotateNinety(0 /*CW*/);
          _step = ATT_ROTATE_CW_1;
        }
      }
      break;

    // ── 6. Rotate 90° CW ─────────────────────────────────────────────
    case ATT_ROTATE_CW_1:
      if (_dt.updateRotateNinety(now)) {
        delay(75);
        Serial.println("[ATT] rotate CW done -> fwd 5");
        _dt.straightMove(8);
        _step = ATT_FWD_5A;
      }
      break;

    // ── 7. Forward 5 in ──────────────────────────────────────────────
    case ATT_FWD_5A:
      if (_dt.updateStraightMove(now)) {
        Serial.println("[ATT] fwd 5A done -> rotate CCW");
        _dt.rotateNinety(1 /*CCW*/);
        _step = ATT_ROTATE_CCW_1;
      }
      break;

    // ── 8. Rotate 90° CCW ────────────────────────────────────────────
    case ATT_ROTATE_CCW_1:
      if (_dt.updateRotateNinety(now)) {
        delay(75);
        Serial.println("[ATT] rotate CCW done -> fwd 5");
        _dt.straightMove(9);
        _step = ATT_FWD_5B;
      }
      break;

    // ── 9. Forward 5 in ──────────────────────────────────────────────
    case ATT_FWD_5B:
      if (_dt.updateStraightMove(now)) {
        Serial.println("[ATT] fwd 5B done -> rotate CW");
        _dt.rotateNinety(0 /*CW*/);
        _step = ATT_ROTATE_CW_2;
      }
      break;

    // ── 10. Rotate 90° CW ────────────────────────────────────────────
    case ATT_ROTATE_CW_2:
      if (_dt.updateRotateNinety(now)) {
        delay(75);
        Serial.println("[ATT] rotate CW (2) done -> fwd 10");
        _dt.straightMove(20);
        _step = ATT_FWD_10;
      }
      break;

    // ── 11. Forward 10 in ────────────────────────────────────────────
    case ATT_FWD_10:
      if (_dt.updateStraightMove(now)) {
        Serial.println("[ATT] fwd 10 done -> wall follow");
        _dt.rotateNinety(0 /*CW*/);
        _step = ATT_ROTATE_CW_3;
        // _wallFollowStart = millis();
        // _wallFollow.onEnter();
        // _step = ATT_WALL_FOLLOW;
      }
      break;
    
    case ATT_ROTATE_CW_3:
      if (_dt.updateRotateNinety(now)) {
        delay(75);
        Serial.println("[ATT] rotate CW (2) done -> fwd 10");
        _dt.straightMove(9);
        _step = ATT_FWD_9;
      }
      break;
    
    case ATT_FWD_9:
      if (_dt.updateStraightMove(now)) {
        Serial.println("[ATT] fwd 9 done -> wall follow");
        _centeringStart = millis(); 
        _centering.onEnter();
        _centering.setFrontStopThreshold(_frontStopMm);
        _step = ATT_CENTER_2;
      }
      break;
    
    case ATT_CENTER_2:
      _centering.update();

      // Check if 4 seconds (4000ms) have elapsed
      if (millis() - _centeringStart >= 4000) {
        _centering.onExit();
        _centering.setFrontStopThreshold(0.0f); // Reset threshold
        
        Serial.println("[ATT] centering 2 timed out -> pressing nexus");
        _pressesDone = 0;
        _nexusPresser.onEnter();
        _step = ATT_ROTATE_FINAL;
      }
      break;

    // ── 12. Wall follow for _wallFollowMs ────────────────────────────
    // case ATT_WALL_FOLLOW:
    //   _wallFollow.update();
    //   if (millis() - _wallFollowStart >= _wallFollowMs) {
    //     _wallFollow.onExit();
    //     Serial.println("[ATT] wall follow done -> rotate CCW");
    //     _dt.rotateNinety(1 /*CCW*/);
    //     _step = ATT_ROTATE_FINAL;
    //   }
    //   break;

    // ── 13. Rotate 90° CCW ───────────────────────────────────────────
    case ATT_ROTATE_FINAL:
      if (_dt.updateRotateNinety(now)) {
        delay(75);
        Serial.println("[ATT] final rotate done -> final press");
        _finalPresser.onEnter();
        _step = ATT_FINAL_PRESS;
      }
      break;

    // ── 14. Final approach + 8.5s hold + retreat ─────────────────────
    case ATT_FINAL_PRESS:
      _finalPresser.update();
      if (_finalPresser.isDone()) {
        _finalPresser.onExit();
        Serial.println("[ATT] final press done -> DONE");
        _dt.stop();
        _step = ATT_DONE;
      }
      break;

    case ATT_DONE:
      break;
  }
}

void AttackTopTower::onExit() {
  Serial.println("AttackTopTower::onExit");
  switch (_step) {
    case ATT_LOW_TOWER:
      _lowTower.onExit();
      break;
    case ATT_CENTER:
      _centering.onExit();
      _centering.setFrontStopThreshold(0.0f);
      break;
    case ATT_PRESS:
      _nexusPresser.onExit();
      break;
    case ATT_WALL_FOLLOW:
      _wallFollow.onExit();
      break;
    case ATT_FINAL_PRESS:
      _finalPresser.onExit();
      break;
    default:
      break;
  }
  _dt.stop();
  _step = ATT_DONE;
}