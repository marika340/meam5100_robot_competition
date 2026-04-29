#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include "Motor.h"
#include "PID.h"

// =====================================================================
// Drivetrain: differential pair of two Motors plus their per-side PID
// controllers. Owns the manual-warmup -> PID state machine, motor sync,
// direction handling, and a raw-drive escape hatch for autonomous modes.
//
// Two control surfaces:
//
//   1. CLOSED-LOOP RPM (web manual drive):
//        setDirection(L, R)        // -1, 0, +1 each
//        setTargetRPM(rpm)         // kicks off manual warmup
//        update()                  // call every loop; runs warmup->PID
//
//   2. RAW DRIVE (wall-follow, centering, press-button):
//        drive(leftCmd, rightCmd)  // -255..+255 each, bypasses PID
//        stop()                    // coast both motors
//
// Switching between them: any call to drive()/stop() puts the drivetrain
// into IDLE; you must setTargetRPM() again to re-engage closed loop.
// =====================================================================
class Drivetrain {
public:
  enum State { IDLE, MANUAL_WARMUP, PID_RUNNING };

  Drivetrain(Motor& left, Motor& right);

  void  begin();

  // ---- Closed-loop interface --------------------------------------
  void  setPIDGains(float kp, float ki, float kd);
  void  setTargetRPM(float rpm);
  void  setDirection(int leftDir, int rightDir);  // each in {-1,0,1}
  void  setAutoEnable(bool en);
  bool  autoEnabled() const { return _autoEnable; }
  State state()       const { return _state; }
  float targetRPM()   const { return _targetRPM; }

  // Periodic update — run once per main loop iteration.
  void  update();

  // ---- Raw drive (open-loop) interface ----------------------------
  void  drive(int leftCmd, int rightCmd, int maxAbs = 255);
  void  stop();

  // Reset all closed-loop state (called by /mode= switches).
  void  resetClosedLoop();

  Motor& left()  { return _left;  }
  Motor& right() { return _right; }
  PID&   leftPID()  { return _pidL; }
  PID&   rightPID() { return _pidR; }

private:
  Motor& _left;
  Motor& _right;
  PID    _pidL;
  PID    _pidR;

  State  _state;
  bool   _autoEnable;
  float  _targetRPM;       // signed magnitude target (direction in _dir)
  int    _dir[2];          // {-1, 0, +1}
  float  _motorSpeed[2];   // PWM in resolution units (0..res)
  float  _curRPM[2];

  unsigned long _lastPidMs;
  unsigned long _manualStartMs;

  static constexpr float kPidPeriodSec  = 0.100f;   // 100 ms PID tick
  static constexpr float kPidTransition = 0.90f;    // %target to engage
  static constexpr float kSyncGain      = 0.005f;
  static constexpr float kRpmScaleRef   = 130.0f;   // map 0..130 RPM -> 0..res

  void  applyMotor(int idx);
  void  runPidTick(unsigned long nowMs);
  bool  warmupReady(unsigned long nowMs);
  void  seedPidFromManual();
};

#endif // DRIVETRAIN_H
