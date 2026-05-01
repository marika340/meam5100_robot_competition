#ifndef VIVE_H
#define VIVE_H

#include <Arduino.h>
#include "vive510.h"


class vive{
public:
  struct Position {
    float x, y;
  };

  vive(int vivePin);

  bool      begin();
  Position  callibrate();

  Position  getPosition();
  void      setPosition(float x, float y);

private:
  Vive510  _vive;
  Position _currentPosition;

  // Per-instance coordinate state (must NOT be static — each vive
  // object needs its own copy, otherwise two trackers will clobber
  // each other's filtered values).
  uint16_t x, y;
  uint16_t x0, y0, oldx1, oldx2, oldy1, oldy2;

  uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c);
};

#endif
