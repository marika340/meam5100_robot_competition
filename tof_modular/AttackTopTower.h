#ifndef ATTACK_TOP_TOWER_H
#define ATTACK_TOP_TOWER_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"
#include "WallFollow.h"
#include "RobotPosition.h"
#include "ToFArray.h"
#include "PressTower.h"

// =====================================================================
// AttackTopTower: scripted attack on the bridge-side "top tower" button.
//
// Sequence:
//   1. WALL_FOLLOW_TO_BRIDGE — Run WallFollow internally and watch the
//      Vive position. We require N consecutive samples confirming we
//      passed through the bridge entry gateway (≈ 4745.5, 3000 ±200)
//      before we begin watching for the trigger location. This avoids
//      a stray Vive jump near (4329, 2811.5) firing the turn before
//      we've even reached the bridge.
//
//   2. WALL_FOLLOW_ON_BRIDGE — Still wall-following, but now also
//      watching for two things:
//        (a) N consecutive samples confirming the trigger location
//            (4329 ±50, 2811.5 ±50). Firing this advances the state
//            machine to ROTATE.
//        (b) N consecutive samples confirming we crossed the exit
//            gateway (≈ 3622.5, 3000 ±200). This means we overshot
//            the trigger window — abort and let the supervisor route
//            us back to MANUAL_DRIVE.
//
//   3. ROTATE_CCW — Stop wall-following, pivot 90° counter-clockwise
//      in place via Drivetrain::rotateNinety(/*CCW*/ 1).
//
//   4. APPROACH — Drive straight forward at a moderate PWM until the
//      front ToF reads less than _frontStopMm. Mirrors the docking
//      pattern in LowTower / AttackNexus, but without Centering since
//      we are not between two side walls after the CCW turn.
//
//   5. PRESS — Delegate to a PressTower instance configured with an
//      8500 ms hold (passed in by the caller).
//
//   6. DONE — isDone() flips true; supervisor returns to MANUAL_DRIVE.
//
// Composition mirrors LowTower / AttackNexus: AttackTopTower owns no
// PD/PID state itself — it just orchestrates Drivetrain, WallFollow,
// RobotPosition (read-only), ToFArray (read-only), and PressTower.
// =====================================================================
class AttackTopTower : public Mode {
public:
  AttackTopTower(Drivetrain&    dt,
                 WallFollow&    wallFollow,
                 RobotPosition& robotPos,
                 ToFArray&      tofs,
                 PressTower&    presser);

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  bool isDone()  const override { return _step == ATT_DONE; }
  const char* name() const override { return "ATTACK_TOP_TOWER"; }

  // Tunables in case we want to retune from the field without a recompile
  // (left simple for now — wire to the web UI later if needed).
  void setEntryGate(float x, float y, float xTol, float yTol) {
    _entryX = x; _entryY = y; _entryXTol = xTol; _entryYTol = yTol;
  }
  void setTriggerLoc(float x, float y, float xTol, float yTol) {
    _trigX = x; _trigY = y; _trigXTol = xTol; _trigYTol = yTol;
  }
  void setExitGate(float x, float y, float xTol, float yTol) {
    _exitX = x; _exitY = y; _exitXTol = xTol; _exitYTol = yTol;
  }
  void setSampleConfirmCount(uint8_t n) { _confirmN = n; }
  void setApproachPwm(int pwm)          { _approachPwm = pwm; }
  void setFrontStopMm(float mm)         { _frontStopMm = mm; }

private:
  enum Step {
    ATT_WALL_TO_BRIDGE,   // wall-follow, waiting for entry gateway
    ATT_WALL_ON_BRIDGE,   // wall-follow, watching trigger + exit
    ATT_ROTATE_CCW,       // pivoting 90° CCW
    ATT_APPROACH,         // raw drive forward until front ToF stops us
    ATT_PRESS,            // PressTower running
    ATT_DONE
  };

  Drivetrain&    _dt;
  WallFollow&    _wf;
  RobotPosition& _pos;
  ToFArray&      _tofs;
  PressTower&    _presser;

  Step          _step          = ATT_DONE;

  // ---- Vive gating tunables ---------------
  float   _entryX     = 4745.5f, _entryY     = 3000.0f;
  float   _entryXTol  =  300.0f, _entryYTol  =  300.0f;  // x window kept generous so we don't miss

  float   _trigX      = 4189.0f, _trigY      = 3000.0f;
  float   _trigXTol   =   100.0f, _trigYTol   =  300.0f;

  float   _exitX      = 3622.5f, _exitY      = 3000.0f;
  float   _exitXTol   =  300.0f, _exitYTol   =  300.0f;

  // Number of consecutive in-window samples required to fire each gate.
  uint8_t _confirmN   = 5;

  // Running counters of consecutive in-window Vive samples.
  uint8_t _entryHits  = 0;
  uint8_t _trigHits   = 0;
  uint8_t _exitHits   = 0;
  bool    _entryFlag  = false;   // true once the entry gateway was confirmed

  // Vive sampling cadence — RobotPosition::callibrate() runs every loop,
  // but we only count one position sample per _samplePeriodMs to avoid
  // counting the same Vive frame multiple times.
  unsigned long _samplePeriodMs = 30;
  unsigned long _lastSampleMs   = 0;

  // ---- Approach / press tunables ------------------------------------
  int     _approachPwm  = 100;
  float   _frontStopMm  = 100.0f;

  // ---- Helpers ------------------------------------------------------
  bool inWindow(float vx, float vy,
                float cx, float cy, float xTol, float yTol) const {
    return fabsf(vx - cx) <= xTol && fabsf(vy - cy) <= yTol;
  }
  void updateViveCounters();         // called from wall-following steps
  void enterRotate();
  void enterApproach();
  void enterPress();
  void abortToDone(const char* why); // called on exit-gateway overshoot
};

#endif // ATTACK_TOP_TOWER_H
