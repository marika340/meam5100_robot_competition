#include "ViveNavigation.h"

ViveNavigation::ViveNavigation(Drivetrain& dt) : _dt(dt) {}

void ViveNavigation::setTargetVive(uint16_t xVive, uint16_t yVive) {
    NavigationTools::VivePosition v = {(float)xVive, (float)yVive};
    _target = NavigationTools::convertViveToField(v);
    _pendingStart = true;  // flag, don't move yet
    
    Serial.printf("DEBUG: RawVive(%u, %u) -> FieldIn(X: %.2f, Y: %.2f)\n", 
                  xVive, yVive, _target.x, _target.y);
}

void ViveNavigation::update() {
    unsigned long now = millis();

    switch (_step) {
        case NAV_ZONE2_WP:
            if (_dt.updateStraightMove(now)) {
                Serial.println("Reached WP: Turning CW into Zone");
                _dt.rotateNinety(0); // 0 = Clockwise
                _step = NAV_ZONE2_TURN_IN;
            }
            break;
        
        case NAV_ZONE2_TURN_IN:
            if (_dt.updateRotateNinety(now)) {
                float distY = NavigationTools::START_Y_IN - NavigationTools::ZONE2_WAYPOINT_Y;
                Serial.printf("Moving into zone: %.2f in\n", distY);
                _dt.straightMove((int)distY);
                _step = NAV_ZONE2_MOVE_Y;
            }
            break;

        case NAV_ZONE2_MOVE_Y:
            if (_dt.updateStraightMove(now)) {
                Serial.println("In Zone: Turning CCW to face +X");
                _dt.rotateNinety(1); // 1 = Counter-Clockwise
                _step = NAV_ZONE2_ALIGN;
            }
            break;

        case NAV_ZONE2_ALIGN:
            if (_dt.updateRotateNinety(now)) {
                // Now move remaining X from WP_X (10) to Target X
                float remainingX = _target.x - NavigationTools::ZONE2_WAYPOINT_X;
                _dt.straightMove((int)remainingX);
                _step = NAV_MOVE_X;
            }
            break;
        
        case NAV_MOVE_X:
            if (_dt.updateStraightMove(now)) {
                // Rotate 90 CCW
                _dt.rotateNinety(0); 
                _step = NAV_ROTATE;
            }
            break;

        case NAV_ROTATE:
            if (_dt.updateRotateNinety(now)) {
                float currentY = isInZone2(_target.x, _target.y) ? 
                                 NavigationTools::ZONE2_WAYPOINT_Y : 
                                 NavigationTools::START_Y_IN;
                float distY = currentY - _target.y;
                _dt.straightMove((int)distY);
                _step = NAV_MOVE_Y;
            }
            break;

        case NAV_MOVE_Y:
            if (_dt.updateStraightMove(now)) {
                _dt.stop();
                _step = NAV_DONE;
                Serial.println("Nav: Target Reached.");
            }
            break;

        default: break;
    }
}

void ViveNavigation::onEnter() {
    _step = NAV_IDLE;
    if (!_pendingStart) return;
    _pendingStart = false;

    Serial.printf("ViveNav::onEnter encoders L=%ld R=%ld\n",
                  _dt.left().getCount(), _dt.right().getCount());

    if (isInZone2(_target.x, _target.y)) {
        float distToWP = NavigationTools::ZONE2_WAYPOINT_X - NavigationTools::START_X_IN;
        _dt.straightMove((int)distToWP);
        _step = NAV_ZONE2_WP;
    } else {
        float distX = _target.x - NavigationTools::START_X_IN;
        _dt.straightMove((int)distX);
        _step = NAV_MOVE_X;
    }
}

void ViveNavigation::onExit() { _dt.stop(); }

bool ViveNavigation::isInZone2(float x, float y) {
    // If target is in Zone 2 initiate Zone 2 start point logic 
    return (x > -8.0f && x < 8.0f && y < 0.0f); 
}