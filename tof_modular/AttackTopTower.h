#ifndef ATTACK_TOP_TOWER_H
#define ATTACK_TOP_TOWER_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"
#include "LowTower.h"
#include "Centering.h"
#include "PressTower.h"
#include "WallFollow.h"

// =====================================================================
// AttackTopTower: full scripted sequence.
//
//   1.  LowTower (straight + 180° + center + press low tower).
//   2.  Back up _backUpInches.
//   3.  Rotate 180° (two 90° CW rotations).
//   4.  Center + drive until front ToF < _frontStopMm.
//   5.  Press nexus button _pressCount times (nexusPresser).
//   6.  Rotate 90° CW.
//   7.  Drive forward 5 in.
//   8.  Rotate 90° CCW.
//   9.  Drive forward 5 in.
//   10. Rotate 90° CW.
//   11. Drive forward 10 in.
//   12. Wall follow for _wallFollowMs milliseconds.
//   13. Rotate 90° CCW.
//   14. Delegate to finalPresser (approach + 8.5s hold + retreat).
//   15. DONE.
// =====================================================================
class AttackTopTower : public Mode {
public:
  AttackTopTower(Drivetrain& dt,
                 LowTower&   lowTower,
                 Centering&  centering,
                 PressTower& nexusPresser,
                 PressTower& finalPresser,
                 WallFollow& wallFollow,
                 int         backUpInches   = 6,
                 float       frontStopMm    = 100.0f,
                 int         pressCount     = 4,
                 unsigned long wallFollowMs = 5500);

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  bool isDone()  const override { return _step == ATT_DONE; }
  const char* name() const override { return "ATTACK_TOP_TOWER"; }

private:
  enum Step {
    ATT_LOW_TOWER,
    ATT_BACK_UP,
    ATT_ROTATE_180_1,
    ATT_ROTATE_180_2,
    ATT_CENTER,
    ATT_PRESS,
    ATT_ROTATE_CW_1,
    ATT_FWD_5A,
    ATT_ROTATE_CCW_1,
    ATT_FWD_5B,
    ATT_ROTATE_CW_2,
    ATT_FWD_10,
    ATT_ROTATE_CW_3,  // <-- Add this
    ATT_FWD_9,        // <-- Add this
    ATT_CENTER_2,      // <-- Add this
    ATT_WALL_FOLLOW,
    ATT_ROTATE_FINAL,
    ATT_FINAL_PRESS,
    ATT_DONE
  };

  Drivetrain& _dt;
  LowTower&   _lowTower;
  Centering&  _centering;
  PressTower& _nexusPresser;
  PressTower& _finalPresser;
  WallFollow& _wallFollow;

  int           _backUpInches;
  float         _frontStopMm;
  int           _pressCount;
  unsigned long _wallFollowMs;
  unsigned long _centeringStart;

  int           _pressesDone    = 0;
  unsigned long _wallFollowStart = 0;
  Step          _step            = ATT_DONE;
};

#endif // ATTACK_TOP_TOWER_H