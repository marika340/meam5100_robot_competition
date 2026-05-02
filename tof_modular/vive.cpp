#include "vive.h"

vive::vive(int vivePin)
  : _vive(vivePin),
    _currentPosition{0.0f, 0.0f},
    x(0), y(0),
    x0(0), y0(0),
    oldx1(0), oldx2(0),
    oldy1(0), oldy2(0)
{}

bool vive::begin() {
  _vive.begin();
  Serial.println("Vive tracker = started");
  return true;
}

vive::Position vive::getPosition() {
  return _currentPosition;
}

void vive::setPosition(float x, float y) {
  _currentPosition.x = x;
  _currentPosition.y = y;
}

uint32_t vive::med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else
    middle = (a <= b) ? a : b;
  return middle;
}

vive::Position vive::callibrate() {
  if (_vive.status() == VIVE_RECEIVING) {
    oldx2 = oldx1; oldy2 = oldy1;
    oldx1 = x0;    oldy1 = y0;

    x0 = _vive.xCoord();
    y0 = _vive.yCoord();
    x = med3filt(x0, oldx1, oldx2);
    y = med3filt(y0, oldy1, oldy2);
    // Serial.printf("X %d, Y %d\n", x, y);
    if (x > 8000 || y > 8000 || x < 1000 || y < 1000) {
      x = 0; y = 0;
    }
  }
  else {
    // Serial.println("Vive Not Receiving");
    x = 0;
    y = 0;
    _vive.sync(5);
  }
  setPosition((float)x, (float)y);
  return _currentPosition;
}
