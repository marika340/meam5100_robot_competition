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
  float _desiredWallDist  = 55.0f; //CHANGED FROM 60 ORIGINAL, 75 COULD NOT HANDLE CORNERS WELL, 65 sometimes goes to ramp and sometimes avoids ramp, 55 avoids ramp, original: 67
  float _frontStopDist    = 300.0f;  // increased so robot starts turning sooner, wall attack: 280, 275 worked, original: 300, 280, 320
  float _wallLostDist     = 600.0f;
  float _turnClearDist    = 300.0f; // MUST stay > _frontStopDist or handleCorner() pivot loop never runs, wall attack: 285, original: 300, 285, 320
  float _switchMargin     = 40.0f;
  int   _baseSpeed        = 140;
  int   _minSpeed         = 80;
  int   _maxSpeed         = 220;
  int   _sharpTurnOffset  = 75; // 75

  bool          _followRight = false;
  unsigned long _lastLoopMs  = 0;
  unsigned long _loopPeriodMs = 30;

  // ---- Stall recovery ---------------------------------------------
  static constexpr float        STALL_RPM_THRESH   = 5.0f;   // RPM below this = stalled
  static constexpr int          STALL_CMD_THRESH   = 40;     // only watch when commanding >= this
  static constexpr unsigned long STALL_CONFIRM_MS  = 400;    // stall must persist this long
  static constexpr unsigned long STALL_REVERSE_MS  = 600;    // how long to drive backward

  enum class StallState { OK, DETECTING, REVERSING };
  StallState    _stallState     = StallState::OK;
  unsigned long _stallStartMs   = 0;
  unsigned long _reverseStartMs = 0;
  int           _lastLeftCmd    = 0;   // track last commanded speeds for stall check
  int           _lastRightCmd   = 0;

  // ---- Front-ToF stuck detection ----------------------------------
  static constexpr float         TOF_FRONT_STUCK_THRESH     = 15.0f;  // mm — front must change by this to count as "moving"
  static constexpr float         TOF_FRONT_STUCK_MAX        = 700.0f; // it was 600
  static constexpr unsigned long TOF_FRONT_STUCK_CONFIRM_MS = 1000;   // ms  — front must be frozen this long before reversing, it used to be 1500

  float         _lastFrontTofSample = 0.0f;
  unsigned long _frontTofSampleMs   = 0;

  unsigned long _cornerExitMs = 0;
  static constexpr unsigned long RAMP_IMMUNITY_MS = 1000; // it used to be 800

  void updateFollowDirection();
  void handleCorner();         // in-place pivot until front clears
  bool isStalled();            // true when motors are commanded but not moving
  bool isFrontStuck();         // true when front ToF hasn't changed for TOF_FRONT_STUCK_CONFIRM_MS
  void handleStall();          // stall state machine, call at top of update()
};

#endif // WALL_FOLLOW_H
