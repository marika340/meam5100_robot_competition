#ifndef TOF_ARRAY_H
#define TOF_ARRAY_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

// =====================================================================
// ToFArray: three time-of-flight sensors on the shared Wire bus.
//
//   left, right : VL53L0X (range ~1000 mm)
//   front       : VL53L1X (range ~4000 mm)
//
// Handles the XSHUT power-up sequence and per-sensor I2C address
// reassignment, then provides EMA-filtered distance accessors.
//
//   begin()                     init all three; returns false on failure
//   update()                    one pass: read each sensor, update filters
//   left()/front()/right()      filtered mm
//   setAlpha(a)                 EMA weight on new samples (0..1)
// =====================================================================
class ToFArray {
public:
  ToFArray(int xshutLeft, int xshutFront, int xshutRight,
           uint8_t addrLeft, uint8_t addrFront, uint8_t addrRight);

  bool  begin();
  void  update();

  void  setAlpha(float a) { _alpha = a; }

  float left()  const { return _dLeft;  }
  float front() const { return _dFront; }
  float right() const { return _dRight; }

  // Force-reseed the filters (called after begin() to avoid 300mm bias).
  void  primeFilters();

private:
  int     _xshutL, _xshutF, _xshutR;
  uint8_t _addrL,  _addrF,  _addrR;

  Adafruit_VL53L0X _loxLeft;
  Adafruit_VL53L1X _loxFront;
  Adafruit_VL53L0X _loxRight;

  VL53L0X_RangingMeasurementData_t _measL;
  VL53L0X_RangingMeasurementData_t _measR;

  float _dLeft, _dFront, _dRight;
  float _alpha;

  static float ema(float oldVal, float newVal, float a) {
    return a * newVal + (1.0f - a) * oldVal;
  }
  float readL0X(Adafruit_VL53L0X& s, VL53L0X_RangingMeasurementData_t& m);
};

#endif // TOF_ARRAY_H
