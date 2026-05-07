#ifndef PRESS_TOWER_H
#define PRESS_TOWER_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"
#include "ToFArray.h"      
#include "Centering.h" 

// =====================================================================
// PressTower: drives forward, holds against the target, retreats.
//
// Replaces the old delay()-based runPressTower() free function with a
// non-blocking phase machine driven by millis(). Web/sensors/telemetry
// keep ticking during the hold.
//
// Constructor takes phase durations so the same class can serve both
// the long tower hold (~8s) and the short nexus hold (~1.5s):
//   PressTower towerPress(drivetrain, 500, 8000, 500);
//   PressTower nexusPress(drivetrain,  500, 1500, 500);
// =====================================================================
class PressTower : public Mode {
public:
  PressTower(Drivetrain& dt,
             Centering&  centering,
             ToFArray&   tofs,
             unsigned long approachMs,
             unsigned long holdMs,
             unsigned long retreatMs,
             int approachPwm = 80,
             int holdPwm     = 50,
             int retreatPwm  = 80
             );

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  bool isDone()  const override { return _phase == PT_DONE; }
  const char* name() const override { return "PRESS_TOWER"; }

  // Tunables you might want to tweak from the web UI later.
  void setHoldMs(unsigned long ms)     { _holdMs = ms; }
  void setApproachPwm(int p)           { _approachPwm = p; }
  void setHoldPwm(int p)               { _holdPwm = p; }

private:
  enum Phase { PT_APPROACH, PT_CENTERING, PT_HOLD, PT_RETREAT, PT_DONE };

  Drivetrain&   _dt;
  Centering&    _centering;
  ToFArray&     _tofs; 
  Phase         _phase          = PT_DONE;
  unsigned long _phaseStartMs   = 0;

  unsigned long _approachMs;
  unsigned long _holdMs;
  unsigned long _retreatMs;
  int           _approachPwm;
  int           _holdPwm;
  int           _retreatPwm;
};

#endif // PRESS_TOWER_H
