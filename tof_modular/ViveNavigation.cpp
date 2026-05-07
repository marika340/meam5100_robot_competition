#include "ViveNavigation.h"

ViveNavigation::ViveNavigation(Drivetrain& dt) : _dt(dt) {}

void ViveNavigation::setTargetVive(uint16_t xVive, uint16_t yVive) {
    NavigationTools::VivePosition v = {(float)xVive, (float)yVive};
    _target = NavigationTools::convertViveToField(v);
    
    Serial.printf("DEBUG: RawVive(%u, %u) -> FieldIn(X: %.2f, Y: %.2f)\n", 
                  xVive, yVive, _target.x, _target.y);

    // Determine if provided coordinates are in Zone 2
    if (isInZone2(_target.x, _target.y)) {
        Serial.println("ZONE 2 DETECTED: Moving to Waypoint first.");
        float distToWP = NavigationTools::ZONE2_WAYPOINT_X - NavigationTools::START_X_IN;
        Serial.printf("Action: Move X to WP (%.1f inches)\n", distToWP);
        _dt.straightMove((int)distToWP);
        _step = NAV_ZONE2_WP;
    } else {
        // Standard start position
        Serial.println("ZONE 2 CLEAR: Moving directly in X.");
        float distX = _target.x - NavigationTools::START_X_IN;
        Serial.printf("Action: Move X (%.1f inches)\n", distX);
        _dt.straightMove((int)distX);
        _step = NAV_MOVE_X;
    }
}

void ViveNavigation::update() {
    unsigned long now = millis();

    switch (_step) {
        case NAV_ZONE2_WP:
            if (_dt.updateStraightMove(now)) {
                // After reaching Zone 2 Start move remaining X
                Serial.println("Step: REACHED WAYPOINT. Now moving remaining X.");
                float distX = _target.x - NavigationTools::ZONE2_WAYPOINT_X;
                _dt.straightMove((int)distX);
                _step = NAV_MOVE_X;
            }
            break;

        case NAV_MOVE_X:
            if (_dt.updateStraightMove(now)) {
                Serial.printf("X Move Done, starting rotation");
                // Rotate 90 CCW
                _dt.rotateNinety(0); 
                _step = NAV_ROTATE;
            }
            break;

        case NAV_ROTATE:
            if (_dt.updateRotateNinety(now)) {
                Serial.printf("Rotating Done");
                // Face +Y and move DY
                float distY = NavigationTools::START_Y_IN - _target.y;
                _dt.straightMove((int)distY);
                _step = NAV_MOVE_Y;
            }
            break;

        case NAV_MOVE_Y:
            if (_dt.updateStraightMove(now)) {
                Serial.printf("Y Move");
                _dt.stop();
                _step = NAV_DONE;
                Serial.println("Nav: Target Reached.");
            }
            break;

        default: break;
    }
}

void ViveNavigation::onEnter() {
    // This prevents isDone() from being true the moment the mode starts
    if (_step == NAV_DONE || _step == NAV_IDLE) {
        // Only reset if we aren't already mid-maneuver
        _step = NAV_IDLE; 
    }
}

void ViveNavigation::onExit() { _dt.stop(); }

bool ViveNavigation::isInZone2(float x, float y) {
    // If target is in Zone 2 initiate Zone 2 start point logic 
    return (x > -8.0f && x < 8.0f && y > 0.0f); 
}