#include "RobotPosition.h"

RobotPosition::RobotPosition(int leftVivePin, int rightVivePin)
  : left(leftVivePin), right(rightVivePin), medianPosition{0.0f, 0.0f}
{}

bool RobotPosition::begin() {
  left.vive.begin();
  right.vive.begin();
  Serial.println("Vive trackers started");
  return true;
}

RobotPosition::Position RobotPosition::getRobotPosition(VivePos position) {
  switch (position) {
    case LEFT:  return left.position;
    case RIGHT: return right.position;
    case MID:   return medianPosition;
  }
  return medianPosition; // unreachable, keeps the compiler happy
}

void RobotPosition::callibrate() {
  updateTracker(left);
  updateTracker(right);
  medianPosition.x = (left.position.x + right.position.x) / 2.0f;
  medianPosition.y = (left.position.y + right.position.y) / 2.0f;
}

// Reads one Vive, runs the median-of-3 filter on its own history, and
// writes the result into that tracker's Position. Each Tracker carries
// its own oldx/oldy state, so left and right never overwrite each other.
void RobotPosition::updateTracker(Tracker& t) {
  if (t.vive.status() == VIVE_RECEIVING) {
    t.oldx2 = t.oldx1;  t.oldy2 = t.oldy1;
    t.oldx1 = t.x0;     t.oldy1 = t.y0;

    t.x0 = t.vive.xCoord();
    t.y0 = t.vive.yCoord();
    t.x  = med3filt(t.x0, t.oldx1, t.oldx2);
    t.y  = med3filt(t.y0, t.oldy1, t.oldy2);
  } else {
    t.x = 0;
    t.y = 0;
    t.vive.sync(5);
  }
  t.position.x = (float)t.x;
  t.position.y = (float)t.y;
}

uint32_t RobotPosition::med3filt(uint32_t a, uint32_t b, uint32_t c) {
  if ((a <= b) && (a <= c)) return (b <= c) ? b : c;
  if ((b <= a) && (b <= c)) return (a <= c) ? a : c;
  return (a <= b) ? a : b;
}
