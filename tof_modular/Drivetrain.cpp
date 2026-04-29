#include "Drivetrain.h"

Drivetrain::Drivetrain(Motor& left, Motor& right)
  : _left(left), _right(right),
    _pidL(1.4f, 1.0f, 0.0f, 50.0f),
    _pidR(1.4f, 1.0f, 0.0f, 50.0f),
    _state(IDLE),
    _autoEnable(false),
    _targetRPM(0.0f),
    _lastPidMs(0),
    _manualStartMs(0)
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
  _targetRPM     = constrain(rpm, -kRpmScaleRef, kRpmScaleRef);
  _state         = MANUAL_WARMUP;
  _autoEnable    = true;
  _manualStartMs = millis();
  _pidL.reset();
  _pidR.reset();
  // Open-loop PWM proportional to |rpm|, scaled to PWM resolution.
  int  resN  = _left.resolution();
  float magn = constrain(fabsf(rpm), 0.0f, kRpmScaleRef);
  float pwm  = (magn / kRpmScaleRef) * resN;
  _motorSpeed[0] = pwm;
  _motorSpeed[1] = pwm;
  applyMotor(0);
  applyMotor(1);
}

void Drivetrain::setAutoEnable(bool en) {
  _autoEnable = en;
  if (!en) {
    _state = IDLE;
    _pidL.reset();
    _pidR.reset();
  }
}

void Drivetrain::update() {
  if (fabsf(_targetRPM) < 1.0f) {
    // No target — silence everything.
    _state = IDLE;
    _motorSpeed[0] = 0; _motorSpeed[1] = 0;
    _pidL.reset();      _pidR.reset();
    applyMotor(0);      applyMotor(1);
    return;
  }

  unsigned long now = millis();

  switch (_state) {
    case IDLE:
      // Nothing to do — caller controls motors directly via drive().
      break;

    case MANUAL_WARMUP: {
      // Sample RPMs every 100 ms, then check if either we hit the
      // transition threshold or we've timed out.
      if (now - _lastPidMs >= 100) {
        _curRPM[0] = fabsf(_left.computeRPM(now));
        _curRPM[1] = fabsf(_right.computeRPM(now));
        _lastPidMs = now;

        if (_autoEnable && warmupReady(now)) {
          seedPidFromManual();
          _state = PID_RUNNING;
          Serial.println(">>> PID engaged.");
        }
      }
    } break;

    case PID_RUNNING:
      runPidTick(now);
      break;
  }
}

bool Drivetrain::warmupReady(unsigned long nowMs) {
  bool m0ok    = _curRPM[0] >= fabsf(_targetRPM) * kPidTransition;
  bool m1ok    = _curRPM[1] >= fabsf(_targetRPM) * kPidTransition;
  bool timeout = (nowMs - _manualStartMs) > 1000;
  return (m0ok && m1ok) || timeout;
}

void Drivetrain::seedPidFromManual() {
  // Bumpless manual->PID handoff: pre-load each integrator so PID
  // output equals the current open-loop PWM at the moment of handoff.
  int   resN = _left.resolution();
  float pwm  = (constrain(fabsf(_targetRPM), 0.0f, kRpmScaleRef) / kRpmScaleRef) * resN;
  PID*  pids[2] = {&_pidL, &_pidR};
  for (int i = 0; i < 2; i++) {
    float err = fabsf(_targetRPM) - _curRPM[i];
    float seed = (pids[i]->ki() > 0.0f)
               ? (pwm - pids[i]->kp() * err) / pids[i]->ki()
               : 0.0f;
    pids[i]->seedIntegral(seed);
    pids[i]->seedPrevError(err);
  }
  // Reset Motor RPM tracking so first PID dt is sensible.
  _left.computeRPM(millis());
  _right.computeRPM(millis());
}

void Drivetrain::runPidTick(unsigned long nowMs) {
  float dt = (nowMs - _lastPidMs) / 1000.0f;
  if (dt < kPidPeriodSec) return;
  _lastPidMs = nowMs;

  int   resN  = _left.resolution();
  float scale = (float)resN / kRpmScaleRef;   // maps controller units to PWM duty
  float setpt = fabsf(_targetRPM);
  PID*  pids[2] = {&_pidL, &_pidR};
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

void Drivetrain::drive(int leftCmd, int rightCmd, int maxAbs) {
  // Raw drive bypasses the closed-loop state machine.
  _state = IDLE;
  _left.setSpeed(leftCmd,  maxAbs);
  _right.setSpeed(rightCmd, maxAbs);
}

void Drivetrain::stop() {
  _state = IDLE;
  _dir[0] = 0; _dir[1] = 0;
  _motorSpeed[0] = 0; _motorSpeed[1] = 0;
  _left.stop();
  _right.stop();
}

void Drivetrain::resetClosedLoop() {
  _state      = IDLE;
  _autoEnable = false;
  _targetRPM  = 0.0f;
  _dir[0] = 0; _dir[1] = 0;
  _motorSpeed[0] = 0; _motorSpeed[1] = 0;
  _pidL.reset();
  _pidR.reset();
  _left.resetEncoder();
  _right.resetEncoder();
  _manualStartMs = millis();
  _lastPidMs     = millis();
}
