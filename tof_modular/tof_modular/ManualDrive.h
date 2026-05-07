#ifndef MANUAL_DRIVE_H
#define MANUAL_DRIVE_H

#include <Arduino.h>
#include "Mode.h"
#include "Drivetrain.h"

// =====================================================================
// ManualDrive: closed-loop RPM drive controlled by the web UI. Acts as
// the source of truth for "what the user typed at the slider/buttons":
// caches target RPM, direction, and PID gains. When active, mutating a
// cached value is also pushed to Drivetrain immediately so the change
// takes effect mid-drive.
//
// Lifecycle:
//   onEnter() — resetClosedLoop, push cached PID gains, ZERO target/dir
//               (per design: target RPM does NOT persist across mode
//                switches — user must re-touch the slider).
//   update()  — forwards to drivetrain.runPidTick(now)
//   onExit()  — drivetrain.stop()
//
// PID gains DO persist across entries — they're tuning state, not
// drive commands.
// =====================================================================
class ManualDrive : public Mode {
public:
  ManualDrive(Drivetrain& dt);

  void onEnter() override;
  void update()  override;
  void onExit()  override;
  const char* name() const override { return "MANUAL_DRIVE"; }

  // Setters: update cache; if currently active, also push to Drivetrain.
  void setTargetRPM(float rpm);
  void setDirection(int leftDir, int rightDir);
  void setKp(float v);
  void setKi(float v);
  void setKd(float v);

  // Read-only accessors for telemetry / web display
  float kp() const { return _kp; }
  float ki() const { return _ki; }
  float kd() const { return _kd; }
  float targetRPM() const { return _targetRPM; }

private:
  Drivetrain& _dt;
  bool        _active;

  // Drive command state — RESET on each entry.
  float _targetRPM;
  int   _dirL, _dirR;

  // Tuning state — PERSISTS across entries.
  float _kp, _ki, _kd;

  void pushGains();
};

#endif // MANUAL_DRIVE_H
