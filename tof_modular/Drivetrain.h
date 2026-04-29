#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include "Motor.h"
#include "PID.h"

// =====================================================================
// Drivetrain: differential pair of two Motors plus their per-side PID
// controllers. Always closed-loop on RPM during webpage drive; raw PWM
// is the escape hatch for autonomous modes.
//
// Two control surfaces:
//
//   1. CLOSED-LOOP RPM (web manual drive):
//        setDirection(L, R)        // -1, 0, +1 each
//        setTargetRPM(rpm)         // resets PIDs, applies open-loop kick,
//                                  //  then PID takes over after 100 ms
//        update()                  // call every loop; runs PID tick + sync
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
  enum State { IDLE, RUNNING };

  Drivetrain(Motor& left, Motor& right);

  void  begin();

  // ---- Closed-loop interface --------------------------------------
  void  setPIDGains(float kp, float ki, float kd);
  void  setTargetRPM(float rpm);
  void  setDirection(int leftDir, int rightDir);  // each in {-1,0,1}
  State state()     const { return _state; }
  float targetRPM() const { return _targetRPM; }

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
  float  _targetRPM;       // signed magnitude target (direction in _dir)
  int    _dir[2];          // {-1, 0, +1}
  float  _motorSpeed[2];   // PWM in resolution units (0..res)
  float  _curRPM[2];

  unsigned long _lastPidMs;

  static constexpr float kPidPeriodSec  = 0.100f;   // 100 ms PID tick
  static constexpr float kSyncGain      = 0.005f;
  static constexpr float kRpmScaleRef   = 130.0f;   // map 0..130 RPM -> 0..res

  void  applyMotor(int idx);
  void  runPidTick(unsigned long nowMs);
};

#endif // DRIVETRAIN_H
