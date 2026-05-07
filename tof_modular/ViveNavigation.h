#ifndef VIVE_NAVIGATION_H
#define VIVE_NAVIGATION_H

#include "Mode.h"
#include "Drivetrain.h"
#include "NavigationTools.h"

class ViveNavigation : public Mode {
public:
  ViveNavigation(Drivetrain& dt);
  void onEnter() override;
  // Raw vive unit input
  void setTargetVive(uint16_t xVive, uint16_t yVive);

  void update()  override;
  void onExit()  override;
  bool isDone()  const override { return _step == NAV_DONE; }
  const char* name() const override { return "VIVE_NAV"; }

private:
  enum Step { NAV_IDLE, NAV_ZONE2_WP, NAV_MOVE_X, NAV_ROTATE, NAV_MOVE_Y, NAV_DONE };
  Step _step = NAV_IDLE;

  Drivetrain& _dt;
  NavigationTools::FieldPosition _target; // renamed from _targetPos to match .cpp

  bool isInZone2(float x, float y);
};

#endif