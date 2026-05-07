#ifndef ATTACK_NEXUS_H
#define ATTACK_NEXUS_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"
#include "Centering.h"
#include "PressTower.h"
#include "ToFArray.h"

// =====================================================================
// AttackNexus: scripted attack sequence on the nexus button cluster.
//
//   1. Drive straight forward 108 in (dead reckoning via Drivetrain).
//   2. Center between the side walls and drive forward until the front
//      ToF reads "nexus close" (delegated to Centering with a
//      front-stop threshold).
//   3. Press the nexus button (delegated to PressTower in nexus
//      configuration -- short hold). PressTower already approaches,
//      holds, and retreats, so each iteration ends with the robot
//      backed off the button.
//   4. Repeat step 3 four times so all four buttons get hit.
//   5. Final retreat (handled by the last PressTower iteration).
//   6. isDone() flips true; supervisor returns to MANUAL_DRIVE.
//
// Composition mirrors LowTower: AttackNexus owns no PD/PID/timing state
// itself -- it just orchestrates the existing Modes. It reuses the
// shared Centering instance (so /mode=2 and the nexus sequence share
// the same controller) and a PressTower configured with a short hold
// for buttons (the global `nexusPress`).
// =====================================================================
class AttackNexus : public Mode {
public:
  AttackNexus(Drivetrain& dt,
              Centering&  centering,
              PressTower& presser,
              ToFArray&   tofs,
              int         straightInches    = 9 * 12,
              float       frontStopMm       = 100.0f,
              int         pressCount        = 4);

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  bool isDone()  const override { return _step == AN_DONE; }
  const char* name() const override { return "ATTACK_NEXUS"; }

private:
  enum Step { AN_STRAIGHT, AN_CENTER, AN_PRESS, AN_DONE };

  Drivetrain& _dt;
  Centering&  _centering;
  PressTower& _presser;
  ToFArray&   _tofs;

  int   _straightInches;
  float _frontStopMm;
  int   _pressCount;        // total number of presses to perform
  int   _pressesDone = 0;   // running count of completed presses
  Step  _step        = AN_DONE;
};

#endif // ATTACK_NEXUS_H
