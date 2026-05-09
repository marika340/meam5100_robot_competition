#include "Drivetrain.h"

Drivetrain::Drivetrain(Motor& left, Motor& right)
  : _left(left), _right(right),
    _pidL(2.0f, 1.5f, 0.0f, 50.0f),
    _pidR(2.0f, 1.5f, 0.0f, 50.0f),
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
  int dir = (rpm > 0) ? 1 : (rpm < 0) ? -1 : 0;
  _dir[0] = dir;
  _dir[1] = dir;

  _targetRPM = constrain(fabsf(rpm), 0.0f, kRpmScaleRef);
  _pidL.reset();
  _pidR.reset();

  // Open-loop initial kick: PWM proportional to |rpm|. PID will overwrite
  // _motorSpeed on the first 100 ms tick. Without this, motors would idle
  // at zero PWM until the first PID period elapses.
  int   resN = _left.resolution();

  //MANUAL KICKSTART
  int kickPWM = (int)(0.6f * resN); // 60% duty cycle start up
  _motorSpeed[0] = kickPWM;
  _motorSpeed[1] = kickPWM;
  applyMotor(0);
  applyMotor(1);


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

    //STALL FALL BACK
    if (_curRPM[i] < 2.0f && fabsf(_targetRPM) > 10.0f) {
      _motorSpeed[i] = 0.6f * resN; // one-tick burst
      applyMotor(i);
      motors[i]->computeRPM(nowMs);
      continue;                     // skip normal PID this tick
    }

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

// ---- Straight-move tunables -----------------------------------------
namespace {
  constexpr float SM_ONE_ROTATION_IN = 16.02f;   // inches per wheel revolution
  constexpr float SM_COUNTS_PER_REV  = 11.0f * 4 * 21;     // encoder counts per revolution
  constexpr int   SM_BASE_PWM        = 150;      // nominal PWM (0..255)
  constexpr float SCALE              = 3.71f;    // empirical distance fudge factor
  constexpr float SM_KP_SYNC         = 0.5f;     // PWM per count of L-R error
  constexpr unsigned long SM_TICK_MS = 20;       // gate between sync / rotate ticks

  // ---- Rotate-in-place tunables -------------------------------------
  constexpr long  ROT_TARGET_COUNTS = (long)(0.75f * 1632);  // recalibrate
  constexpr int   ROT_PWM           = 150;
}

void Drivetrain::straightMove(int desiredDistInches) {
  // Kick off a non-blocking dead-reckoning straight move.
  // desiredDist is inches; sign sets direction (+ = forward, - = backward).
  // The actual driving happens in updateStraightMove(), which the main
  // loop must call every iteration.
  if (desiredDistInches == 0) { stop(); _smActive = false; return; }

  _smActive       = true;
  _smStartTime = millis();
  
  _smDir = (desiredDistInches >= 0) ? 1 : -1;
  float tpi = 184.0;

  if (desiredDistInches > 24) {
    tpi = 213.0;
  }

  _smTargetCounts = (long)(fabsf((float)desiredDistInches) * tpi);

  // Safety: If the math fails, don't let it be 0
  if (_smTargetCounts < 10) _smTargetCounts = 500; 

  Serial.printf("DEBUG: Moving %d inches. Target Ticks: %ld\n", desiredDistInches, _smTargetCounts);

  _smLeftStart    = _left.getCount();
  _smRightStart   = _right.getCount();
  _smLastTickMs   = 0;

  _targetRPM = 0.0f;
  _pidL.reset();
  _pidR.reset();

  _left.setSpeed(_smDir  * 150, 255);
  _right.setSpeed(_smDir * 150, 255);
}

bool Drivetrain::updateStraightMove(unsigned long nowMs) {
  if (!_smActive) return true;     // nothing to do = "done"

  // Rate-limit the sync update so KP_SYNC tuning doesn't depend on how
  // fast the main loop happens to spin.
  //if (nowMs - _smLastTickMs < SM_TICK_MS) return false;
  //_smLastTickMs = nowMs;

  long lCounts = labs(_left.getCount()  - _smLeftStart);
  long rCounts = labs(_right.getCount() - _smRightStart);


  if (lCounts >= _smTargetCounts || rCounts >= _smTargetCounts) {
    _smActive = false;
    stop();
    return true;
  }

  // P-sync: if left has rolled further than right, slow left, boost right.
  long err  = lCounts - rCounts;
  int  corr = (int)(SM_KP_SYNC * (float)err);
  corr = constrain(corr, -SM_BASE_PWM, SM_BASE_PWM);

  int leftCmd  = _smDir * (SM_BASE_PWM - corr);
  int rightCmd = _smDir * (SM_BASE_PWM + corr);

  // Cut whichever side has hit target so the other can catch up.
  if (lCounts >= _smTargetCounts) leftCmd  = 0;
  if (rCounts >= _smTargetCounts) rightCmd = 0;

  _left.setSpeed(leftCmd,  255);
  _right.setSpeed(rightCmd, 255);
  return false;
}

void Drivetrain::rotateNinety(int dir) {
  // Kick off a non-blocking in-place 90° pivot. dir==0 -> CW, else CCW.
  // Driving happens in updateRotateNinety(), which the main loop must
  // call every iteration until it returns true.
  _rotDir          = dir;
  _rotTargetCounts = ROT_TARGET_COUNTS;
  _rotLeftStart    = _left.getCount();
  _rotRightStart   = _right.getCount();
  _rotLastTickMs   = 0;
  _rotActive       = true;

  // Clear any closed-loop state — we're driving raw PWM.
  _targetRPM = 0.0f;
  _pidL.reset();
  _pidR.reset();

  // Counter-rotate the two sides immediately so the move starts on this tick.
  if (dir == 0) {  // CW
    _left.setSpeed( ROT_PWM, 255);
    _right.setSpeed(-ROT_PWM, 255);
  } else {         // CCW
    _left.setSpeed(-ROT_PWM, 255);
    _right.setSpeed( ROT_PWM, 255);
  }
}

bool Drivetrain::updateRotateNinety(unsigned long nowMs) {
  if (!_rotActive) return true;     // nothing to do = "done"

  // Rate-limit so this doesn't depend on loop speed.
  if (nowMs - _rotLastTickMs < SM_TICK_MS) return false;
  _rotLastTickMs = nowMs;

  long lCounts = labs(_left.getCount()  - _rotLeftStart);
  long rCounts = labs(_right.getCount() - _rotRightStart);

  if (lCounts >= _rotTargetCounts && rCounts >= _rotTargetCounts) {
    _rotActive = false;
    stop();
    return true;
  }
  // No PWM updates needed — both motors were set in rotateNinety()
  // and run open-loop until target counts are reached.
  return false;
}

void Drivetrain::stop() {
  // "Stop and forget": clear target and direction so any later
  // accidental runPidTick() is a no-op. Caller must re-setTargetRPM
  // to engage closed-loop again.
  _targetRPM = 0.0f;
  _dir[0] = 0; _dir[1] = 0;
  _motorSpeed[0] = 0; _motorSpeed[1] = 0;
  _smActive  = false;           // abort any in-progress straight move
  _rotActive = false;           // abort any in-progress rotate
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