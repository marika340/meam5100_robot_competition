#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

#include <Arduino.h>
#include "vive510.h"

// Which tracker (or the midpoint) to query via getRobotPosition().
// File-scope so callers can write LEFT / RIGHT / MID directly.
enum VivePos { LEFT, RIGHT, MID };

class RobotPosition {
public:
  struct Position { float x, y; };

  RobotPosition(int leftVivePin, int rightVivePin);

  bool     begin();
  void     callibrate();
  Position getRobotPosition(VivePos position);

private:
  // One bundle of "Vive + its own filter history + its own filtered
  // position" per tracker. Two of these as members means the two
  // trackers can't clobber each other's filter state.
  struct Tracker {
    Vive510  vive;
    Position position;
    uint16_t x  = 0, y  = 0;
    uint16_t x0 = 0, y0 = 0;
    uint16_t oldx1 = 0, oldx2 = 0;
    uint16_t oldy1 = 0, oldy2 = 0;

    Tracker(int pin) : vive(pin), position{0.0f, 0.0f} {}
  };

  Tracker  left, right;
  Position medianPosition;

  void     updateTracker(Tracker& t);
  uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c);
};

#endif
