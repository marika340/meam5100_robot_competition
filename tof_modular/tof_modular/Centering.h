#ifndef CENTERING_H
#define CENTERING_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"
#include "ToFArray.h"
#include "PID.h"

// =====================================================================
// Centering: drives forward while keeping equal distance to the left
// and right walls. PD on (right - left) ToF difference; the controller
// output is added/subtracted to a base PWM and pushed via raw drive.
//
// Two activation modes:
//   1. Open-ended (default) — runs until the supervisor switches it
//      out. Use this for the "Centering" web button.
//   2. Stop-on-front — set setFrontStopThreshold(mm > 0) before
//      onEnter(). When the front ToF reads less than that threshold,
//      isDone() flips to true and the supervisor (or a composite mode)
//      can transition out. Use this when chained into a sequence
//      (e.g. LowTower: drive centered until you're close to the tower).
//
// PID + lastMs are member fields (not function-locals) so multiple
// activations work cleanly.
// =====================================================================
class Centering : public Mode {
public:
  Centering(Drivetrain& dt, ToFArray& tofs);

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  bool isDone()  const override { return _done; }
  const char* name() const override { return "CENTERING"; }

  // 0 (default) = no auto-stop. >0 = stop when tofs.front() < mm.
  void setFrontStopThreshold(float mm) { _frontStopThreshold = mm; }
  void setBaseSpeed(int s)             { _baseSpeed = s; }
  void setKp(float v)                  { _pid.setGains(v, _pid.ki(), _pid.kd()); _pid.reset(); }
  void setKd(float v)                  { _pid.setGains(_pid.kp(), _pid.ki(), v); _pid.reset(); }

private:
  Drivetrain& _dt;
  ToFArray&   _tofs;
  PID         _pid;

  unsigned long _lastMs            = 0;
  int           _baseSpeed         = 140;
  float         _frontStopThreshold = 0.0f;   // 0 disables
  bool          _done              = false;

  // PD on lateral error: ignore noise inside the deadband.
  static constexpr float DEADBAND_MM   = 10.0f;
  static constexpr float CTRL_LIMIT    = 100.0f;
  static constexpr float MIN_DT_SEC    = 0.01f;
};

#endif // CENTERING_H
