#include "Drivetrain.h"

Drivetrain::Drivetrain(Motor& left, Motor& right)
  : _left(left), _right(right),
    _pidL(1.4f, 1.0f, 0.0f, 50.0f),
    _pidR(1.4f, 1.0f, 0.0f, 50.0f),
    _targetRPM(0.0f),
    _lastPidMs(0)
{
  _dir[0] = 0; _dir[1] = 0;
  _motorSpeed[0] = 0; _motorSpeed[1] = 0;
  _curRPM[0] = 0; _curRPM[1] = 0;
}

void Drivetrain::begin() {
  _left.begin();
  _right.begin();
  _lastPidMs = millis();
}

void Drivetrain::setPIDGains(float kp, float ki, float kd) {
  _pidL.setGains(kp, ki, kd);
  _pidR.setGains(kp, ki, kd);
  _pidL.reset();
  _pidR.reset();
}

void Drivetrain::setDirection(int leftDir, int rightDir) {
  // Brief stop before reversing direction (matches original handleDir
  // behaviour — protects H-bridge / motor from instant flip).
  _dir[0] = 0; _dir[1] = 0;
  _motorSpeed[0] = 0; _motorSpeed[1] = 0;
  applyMotor(0); applyMotor(1);
  delay(10);

  _dir[0] = constrain(leftDir,  -1, 1);
  _dir[1] = constrain(rightDir, -1, 1);
  _pidL.reset();
  _pidR.reset();
  applyMotor(0);
  applyMotor(1);
}

void Drivetrain::setTargetRPM(float rpm) {
  _targetRPM = constrain(rpm, -kRpmScaleRef, kRpmScaleRef);
  _pidL.reset();
  _pidR.reset();

  // Open-loop initial kick: PWM proportional to |rpm|. PID will overwrite
  // _motorSpeed on the first 100 ms tick. Without this, motors would idle
  // at zero PWM until the first PID period elapses.
  int   resN = _left.resolution();
  float magn = constrain(fabsf(rpm), 0.0f, kRpmScaleRef);
  float pwm  = (magn / kRpmScaleRef) * resN;
  _motorSpeed[0] = pwm;
  _motorSpeed[1] = pwm;

  // Prime encoder RPM trackers so the first PID dt is sane.
  unsigned long now = millis();
  _left.computeRPM(now);
  _right.computeRPM(now);
  _lastPidMs = now;

  applyMotor(0);
  applyMotor(1);
}

void Drivetrain::runPidTick(unsigned long nowMs) {
  // No target — keep motors stopped and PID state clean.
  if (fabsf(_targetRPM) < 1.0f) {
    _motorSpeed[0] = 0; _motorSpeed[1] = 0;
    _pidL.reset();      _pidR.reset();
    applyMotor(0);      applyMotor(1);
    return;
  }

  float dt = (nowMs - _lastPidMs) / 1000.0f;
  if (dt < kPidPeriodSec) return;
  _lastPidMs = nowMs;

  int   resN  = _left.resolution();
  float scale = (float)resN / kRpmScaleRef;   // controller units -> PWM duty
  float setpt = fabsf(_targetRPM);
  PID*  pids[2]   = {&_pidL, &_pidR};
  Motor* motors[2] = {&_left, &_right};

  for (int i = 0; i < 2; i++) {
    if (_dir[i] == 0) {
      _motorSpeed[i] = 0;
      pids[i]->reset();
      applyMotor(i);
      continue;
    }
    _curRPM[i]     = fabsf(motors[i]->computeRPM(nowMs));
    float ctrl     = pids[i]->compute(setpt, _curRPM[i], dt);
    _motorSpeed[i] = constrain(ctrl * scale, 0.0f, (float)resN);
  }

  // ---- Sync: pull the two sides toward equal RPM -----------------
  if (_dir[0] != 0 && _dir[1] != 0) {
    float syncErr = _curRPM[0] - _curRPM[1];
    _motorSpeed[0] = constrain(_motorSpeed[0] - kSyncGain * scale * syncErr, 0.0f, (float)resN);
    _motorSpeed[1] = constrain(_motorSpeed[1] + kSyncGain * scale * syncErr, 0.0f, (float)resN);

    // Diffuse integrators a little so they don't drift apart forever.
    float iDiff = _pidL.integral() - _pidR.integral();
    _pidL.seedIntegral(_pidL.integral() - 0.1f * iDiff);
    _pidR.seedIntegral(_pidR.integral() + 0.1f * iDiff);
  }

  applyMotor(0);
  applyMotor(1);

  Serial.printf("[DT] tgt:%.1f L:%.1f R:%.1f pwmL:%.0f pwmR:%.0f\n",
                setpt, _curRPM[0], _curRPM[1], _motorSpeed[0], _motorSpeed[1]);
}

void Drivetrain::applyMotor(int idx) {
  Motor& m = (idx == 0) ? _left : _right;
  if (_dir[idx] == 0) { m.stop(); return; }
  // _motorSpeed is a magnitude in PWM-resolution units; pass to Motor
  // with maxAbs = resolution so it writes the duty unchanged.
  int signedDuty = (int)(_dir[idx] * _motorSpeed[idx]);
  m.setSpeed(signedDuty, m.resolution());
}

void Drivetrain::driveDirect(int leftCmd, int rightCmd, int maxAbs) {
  // Raw open-loop drive. Caller (typically an autonomous Mode) is fully
  // responsible for sequencing; we don't touch closed-loop state here.
  _left.setSpeed(leftCmd,  maxAbs);
  _right.setSpeed(rightCmd, maxAbs);
}

void Drivetrain::stop() {
  // "Stop and forget": clear target and direction so any later
  // accidental runPidTick() is a no-op. Caller must re-setTargetRPM
  // to engage closed-loop again.
  _targetRPM = 0.0f;
  _dir[0] = 0; _dir[1] = 0;
  _motorSpeed[0] = 0; _motorSpeed[1] = 0;
  _left.stop();
  _right.stop();
}

void Drivetrain::resetClosedLoop() {
  _targetRPM = 0.0f;
  _dir[0] = 0; _dir[1] = 0;
  _motorSpeed[0] = 0; _motorSpeed[1] = 0;
  _pidL.reset();
  _pidR.reset();
  _left.resetEncoder();
  _right.resetEncoder();
  _lastPidMs = millis();
}
