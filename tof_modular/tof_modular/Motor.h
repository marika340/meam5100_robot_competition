#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Encoder.h"

// =====================================================================
// Motor: one DC motor with H-bridge + quadrature encoder.
//
// Owns its PWM pin, two direction pins, a PWM ledc channel, and an
// Encoder instance. Exposes a single signed-speed interface; direction
// and PWM mapping are handled internally.
//
//   setSpeed(s)  s in [-maxAbs, +maxAbs]; sign determines direction
//   stop()       coast (both dir lines LOW, PWM 0)
//   getCount()   raw encoder count
//   computeRPM(now)
//                returns RPM since last call; updates internal state
//   resetEncoder()
//
// The PID controller is intentionally NOT inside Motor — Motor is a
// dumb actuator + sensor pair. Drivetrain owns the controller.
// =====================================================================
class Motor {
public:
  // pwmPin       : H-bridge PWM input
  // dir1, dir2   : H-bridge direction lines
  // encA, encB   : quadrature encoder pins
  // pwmChannel   : ledc channel index (0..15)
  // freqHz       : PWM frequency
  // resBits      : PWM resolution in bits (e.g. 14 -> 16383)
  // countsPerRev : encoder counts per output-shaft revolution
  Motor(int pwmPin, int dir1, int dir2,
        int encA, int encB,
        int pwmChannel,
        int freqHz, int resBits,
        float countsPerRev);

  void  begin();
  void  setSpeed(int signedSpeed, int maxAbs = 255);
  void  stop();

  long  getCount();
  void  resetEncoder();
  // Returns RPM since the last call. First call after begin() returns 0.
  float computeRPM(unsigned long nowMs);

  int   resolution() const { return _resolution; }

private:
  int     _pwmPin, _dir1, _dir2;
  int     _encA,   _encB;
  int     _pwmChannel;
  int     _freqHz, _resBits;
  int     _resolution;       // (1 << resBits) - 1
  float   _countsPerRev;

  Encoder _enc;

  long          _prevCount;
  unsigned long _prevTimeMs;
  bool          _rpmInit;
};

#endif // MOTOR_H
