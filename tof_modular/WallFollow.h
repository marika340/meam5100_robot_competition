#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"
#include "ToFArray.h"
#include "PID.h"

// =====================================================================
// WallFollow: autonomous 3-ToF wall-following Mode.
//
// On entry, picks the closer of the two side walls to follow. Drives
// at baseSpeed and corrects with a PD controller on the side-wall
// distance error. When the front sensor sees an obstacle closer than
// frontStopDist, performs an in-place pivot until the front clears
// past turnClearDist.
//
// Tunable from the web UI: wf_Kp, wf_Kd, sharpTurnOffset (and others
// can be exposed the same way).
// =====================================================================
class WallFollow : public Mode {
public:
  WallFollow(Drivetrain& dt, ToFArray& tof);

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  const char* name() const override { return "WALL_FOLLOWING"; }

  // ---- Tunables exposed to WebController --------------------------
  void setKp(float v)            { _pid.setGains(v, 0.0f, _pid.kd()); _pid.reset(); }
  void setKd(float v)            { _pid.setGains(_pid.kp(), 0.0f, v); _pid.reset(); }
  void setSharpTurnOffset(int v) { _sharpTurnOffset = constrain(v, 0, 255); }
  void setBaseSpeed(int v)       { _baseSpeed = v; }
  void setFollowRightWall(bool v){ _followRight = v; }

  float kp() const               { return _pid.kp(); }
  float kd() const               { return _pid.kd(); }
  int   sharpTurnOffset() const  { return _sharpTurnOffset; }

private:
  Drivetrain& _dt;
  ToFArray&   _tof;
  PID         _pid;             // pure PD, ki=0

  // Geometry / tuning
  float _desiredWallDist  = 60.0f;
  float _frontStopDist    = 280.0f;
  float _wallLostDist     = 600.0f;
  float _turnClearDist    = 280.0f;
  float _switchMargin     = 40.0f;
  int   _baseSpeed        = 140;
  int   _minSpeed         = 80;
  int   _maxSpeed         = 220;
  int   _sharpTurnOffset  = 75;

  bool          _followRight = false;
  unsigned long _lastLoopMs  = 0;
  unsigned long _loopPeriodMs = 30;

  void updateFollowDirection();
  void handleCorner();    // in-place pivot until front clears
};

#endif // WALL_FOLLOW_H
