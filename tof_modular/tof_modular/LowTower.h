#ifndef LOW_TOWER_H
#define LOW_TOWER_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"
#include "Centering.h"
#include "PressTower.h"
#include "ToFArray.h"

// =====================================================================
// LowTower: scripted attack sequence on the low tower.
//
//   1. Drive straight forward 9 ft (dead reckoning via Drivetrain).
//   2. Rotate 90° in place, twice (180° total — robot is now facing
//      back toward the field interior).
//   3. Center between the side walls and drive forward until the front
//      ToF reads "tower close" (delegated to Centering with a
//      front-stop threshold).
//   4. Press the tower (delegated to PressTower).
//   5. isDone() flips true; supervisor returns to MANUAL_DRIVE.
//
// Composition over inheritance: LowTower owns no PD/PID/timing state
// itself — it just orchestrates the existing Modes. The Centering and
// PressTower instances are shared with the supervisor (so /mode=2 and
// /mode=3 reuse the same Centering instance), which is fine because
// only one supervisor mode is active at a time.
// =====================================================================
class LowTower : public Mode {
public:
  LowTower(Drivetrain& dt,
           Centering&  centering,
           PressTower& presser,
           ToFArray&   tofs,
           int         straightInches    = 6 * 12, //SHORTED DEADRECKNONING TO MAKE IT MORE ROBUST WITH TOF INTERGRATION
           float       frontStopMm       = 100.0f,
           int         rotateDir         = 0);   // 0 -> CW, else CCW

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  bool isDone()  const override { return _step == LT_DONE; }
  const char* name() const override { return "LOW_TOWER"; }

private:
  enum Step { LT_STRAIGHT, LT_ROTATE_1, LT_ROTATE_2, LT_CENTER, LT_PRESS, LT_DONE };

  Drivetrain& _dt;
  Centering&  _centering;
  PressTower& _presser;
  ToFArray&   _tofs;
  int   _straightInches;
  float _frontStopMm;
  int   _rotateDir;
  Step  _step = LT_DONE;
};

#endif // LOW_TOWER_H
