#include "ToFArray.h"

ToFArray::ToFArray(int xshutLeft, int xshutFront, int xshutRight,
                   uint8_t addrLeft, uint8_t addrFront, uint8_t addrRight)
  : _xshutL(xshutLeft),  _xshutF(xshutFront),  _xshutR(xshutRight),
    _addrL(addrLeft),    _addrF(addrFront),    _addrR(addrRight),
    _dLeft(300.0f), _dFront(300.0f), _dRight(300.0f),
    _alpha(0.35f)
{}

float ToFArray::readL0X(Adafruit_VL53L0X& s, VL53L0X_RangingMeasurementData_t& m) {
  s.rangingTest(&m, false);
  if (m.RangeStatus == 4 || m.RangeMilliMeter <= 0) return 1200.0f;
  return (float)m.RangeMilliMeter;
}

bool ToFArray::begin() {
  pinMode(_xshutL, OUTPUT);
  pinMode(_xshutF, OUTPUT);
  pinMode(_xshutR, OUTPUT);
  digitalWrite(_xshutL, LOW);
  digitalWrite(_xshutF, LOW);
  digitalWrite(_xshutR, LOW);
  delay(100);

  // ---- LEFT VL53L0X --------------------------------------------------
  digitalWrite(_xshutL, HIGH); delay(50);
  if (!_loxLeft.begin(_addrL, false, &Wire)) {
    Serial.println("Failed: LEFT VL53L0X");
    return false;
  }
  Serial.println("LEFT VL53L0X OK");

  // ---- RIGHT VL53L0X -------------------------------------------------
  digitalWrite(_xshutR, HIGH); delay(50);
  if (!_loxRight.begin(_addrR, false, &Wire)) {
    Serial.println("Failed: RIGHT VL53L0X");
    return false;
  }
  Serial.println("RIGHT VL53L0X OK");

  // ---- FRONT VL53L1X -------------------------------------------------
  digitalWrite(_xshutF, HIGH); delay(100);
  if (!_loxFront.begin(_addrF, &Wire)) {
    Serial.println("Failed: FRONT VL53L1X");
    return false;
  }
  Serial.println("FRONT VL53L1X OK");

  if (!_loxFront.startRanging()) {
    Serial.print(F("FRONT ToF: couldn't start ranging: "));
    Serial.println(_loxFront.vl_status);
    return false;
  }
  // Valid timing budgets for VL53L1X: 15, 20, 33, 50, 100, 200, 500 ms.
  _loxFront.setTimingBudget(50);
  Serial.print(F("Front ToF timing budget (ms): "));
  Serial.println(_loxFront.getTimingBudget());

  return true;
}

void ToFArray::primeFilters() {
  // Take one fresh sample on each sensor and overwrite the filters so
  // the first few update() passes don't drag values toward 300 mm.
  _dLeft  = readL0X(_loxLeft,  _measL);
  _dRight = readL0X(_loxRight, _measR);
  if (_loxFront.dataReady()) {
    int16_t d = _loxFront.distance();
    if (d > 0) _dFront = (float)d;
    _loxFront.clearInterrupt();
  }
}

void ToFArray::update() {
  // VL53L0X side sensors block-poll
  _dLeft  = ema(_dLeft,  readL0X(_loxLeft,  _measL), _alpha);
  _dRight = ema(_dRight, readL0X(_loxRight, _measR), _alpha);

  // VL53L1X is interrupt/data-ready driven
  if (_loxFront.dataReady()) {
    float front = (float)_loxFront.distance();
    if (front > 0) _dFront = ema(_dFront, front, _alpha);
    _loxFront.clearInterrupt();
  }
}
