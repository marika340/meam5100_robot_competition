#include "Motor.h"

Motor::Motor(int pwmPin, int dir1, int dir2,
             int encA, int encB,
             int pwmChannel,
             int freqHz, int resBits,
             float countsPerRev)
  : _pwmPin(pwmPin), _dir1(dir1), _dir2(dir2),
    _encA(encA), _encB(encB),
    _pwmChannel(pwmChannel),
    _freqHz(freqHz), _resBits(resBits),
    _resolution((1 << resBits) - 1),
    _countsPerRev(countsPerRev),
    _enc(encA, encB),
    _prevCount(0),
    _prevTimeMs(0),
    _rpmInit(false)
{}

void Motor::begin() {
  pinMode(_dir1, OUTPUT);
  pinMode(_dir2, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
  pinMode(_encA, INPUT_PULLUP);
  pinMode(_encB, INPUT_PULLUP);
  ledcAttachChannel(_pwmPin, _freqHz, _resBits, _pwmChannel);
  digitalWrite(_dir1, LOW);
  digitalWrite(_dir2, LOW);
  ledcWrite(_pwmPin, 0);
}

// function that receives the desired speed and sends signal to driver
void Motor::setSpeed(int signedSpeed, int maxAbs) {
  signedSpeed = constrain(signedSpeed, -maxAbs, maxAbs);
  if (signedSpeed > 0) {
    digitalWrite(_dir1, LOW);
    digitalWrite(_dir2, HIGH);
    ledcWrite(_pwmPin, map(signedSpeed, 0, maxAbs, 0, _resolution));
  } else if (signedSpeed < 0) {
    digitalWrite(_dir1, HIGH);
    digitalWrite(_dir2, LOW);
    ledcWrite(_pwmPin, map(-signedSpeed, 0, maxAbs, 0, _resolution));
  } else {
    digitalWrite(_dir1, LOW);
    digitalWrite(_dir2, LOW);
    ledcWrite(_pwmPin, 0);
  }
}

void Motor::stop() {
  digitalWrite(_dir1, LOW);
  digitalWrite(_dir2, LOW);
  ledcWrite(_pwmPin, 0);
}

long Motor::getCount() {
  return _enc.read();
}

void Motor::resetEncoder() {
  _enc.write(0);
  _prevCount  = 0;
  _prevTimeMs = millis();
  _rpmInit    = false;
}

float Motor::computeRPM(unsigned long nowMs) {
  if (!_rpmInit) {
    _prevCount  = _enc.read();
    _prevTimeMs = nowMs;
    _rpmInit    = true;
    return 0.0f;
  }
  long  curCount = _enc.read();
  float dt       = (nowMs - _prevTimeMs) / 1000.0f;
  if (dt <= 0.0f) return 0.0f;
  float rpm = ((curCount - _prevCount) / _countsPerRev) * (60.0f / dt);
  _prevCount  = curCount;
  _prevTimeMs = nowMs;
  return rpm;
}
