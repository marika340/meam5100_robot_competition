#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include "Motor.h"
#include "PID.h"

// Drivetrain: low-level service. Owns two Motors and two per-side PID
// controllers, plus the primitives needed by Modes that drive the car.
// Drivetrain has NO opinion about *which* mode is active — Modes decide
// when to call setTargetRPM / runPidTick vs. driveDirect.
//
// Two control surfaces:
//
//   1. CLOSED-LOOP RPM (used by ManualDrive, future autonomous modes):
//        setDirection(L, R)        // -1, 0, +1 each
//        setTargetRPM(rpm)         // open-loop kick + PID arms
//        setPIDGains(kp, ki, kd)
//        runPidTick(now)           // call every loop tick; gated to
//                                  //  100 ms internally
//
//   2. RAW DRIVE (used by WallFollow, centering, press-button):
//        driveDirect(L, R)         // -255..+255 each, bypasses PID
//        stop()                    // coast both motors, clear target
//
// resetClosedLoop() wipes target/direction/PID state and zeros encoders.
// Called by ManualDrive::onEnter for a clean slate.
class Drivetrain {
public:
  Drivetrain(Motor& left, Motor& right);

  void  begin();

  // ---- Closed-loop primitives -------------------------------------
  void  setPIDGains(float kp, float ki, float kd);
  void  setTargetRPM(float rpm);
  void  setDirection(int leftDir, int rightDir);  // each in {-1,0,1}
  void  runPidTick(unsigned long nowMs);          // gated to 100 ms

  // ---- Raw drive (open-loop) primitives ---------------------------
  void  driveDirect(int leftCmd, int rightCmd, int maxAbs = 255);

  // ---- Non-blocking straight-move (dead reckoning) ----------------
  // Usage:
  //   drivetrain.straightMove(48);              // kick off (inches; sign = dir)
  //   while (...) {
  //     if (drivetrain.updateStraightMove(millis())) { /* done */ }
  //     web.serve(); tofs.update(); ...         // main loop keeps running
  //   }
  void  straightMove(int desiredDist);
  bool  updateStraightMove(unsigned long nowMs);   // returns true when done (or inactive)
  bool  isStraightMoveActive() const { return _smActive; }

  // ---- Non-blocking in-place 90° pivot ----------------------------
  // Same shape as straightMove: kick + poll. dir==0 -> CW, anything else -> CCW.
  // (Naming is historical; if you want clarity, switch callers to an enum.)
  void  rotateNinety(int dir);
  bool  updateRotateNinety(unsigned long nowMs);   // returns true when done (or inactive)
  bool  isRotateActive() const { return _rotActive; }

  void  stop();

  // Wipe all closed-loop state + reset encoders.
  void  resetClosedLoop();

  // Accessors (read-only, useful for telemetry or future PID consumers)
  Motor& left()  { return _left;  }
  Motor& right() { return _right; }
  PID&   leftPID()  { return _pidL; }
  PID&   rightPID() { return _pidR; }
  float  targetRPM() const { return _targetRPM; }

  // Last measured RPM per side (updated by runPidTick; 0 when not in
  // closed-loop mode, but still valid for stall detection since
  // driveDirect callers can call these after a computeRPM snapshot).
  float  leftRPM()  const { return _curRPM[0]; }
  float  rightRPM() const { return _curRPM[1]; }

  const float WheelDiam    = 1.6f;
  const float TicksPerRev  = 184.0f;
  const float TicksPerInch = TicksPerRev / (WheelDiam * 3.14159f);

private:
  Motor& _left;
  Motor& _right;
  PID    _pidL;
  PID    _pidR;

  float  _targetRPM;       // signed magnitude target (direction in _dir)
  int    _dir[2];          // {-1, 0, +1}
  float  _motorSpeed[2];   // PWM in resolution units (0..res)
  float  _curRPM[2];

  unsigned long _lastPidMs;

  // ---- Straight-move state (non-blocking) -------------------------
  bool          _smActive       = false;
  int           _smDir          = 0;       // +1 forward, -1 backward
  long          _smTargetCounts = 0;
  long          _smLeftStart    = 0;
  long          _smRightStart   = 0;
  unsigned long _smLastTickMs   = 0;
  unsigned long _smStartTime    = 0;   

  // ---- Rotate-90 state (non-blocking) -----------------------------
  bool          _rotActive      = false;
  int           _rotDir         = 0;       // 0 -> CW (matches old API)
  long          _rotTargetCounts = 0;
  long          _rotLeftStart   = 0;
  long          _rotRightStart  = 0;
  unsigned long _rotLastTickMs  = 0;

  static constexpr float kPidPeriodSec = 0.100f;   // 100 ms PID tick
  static constexpr float kSyncGain     = 0.005f;
  static constexpr float kRpmScaleRef  = 130.0f;   // map 0..130 RPM -> 0..res

  void  applyMotor(int idx);   // private: internal dir+magnitude → Motor
};

#endif // DRIVETRAIN_H
