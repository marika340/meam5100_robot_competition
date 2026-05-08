#ifndef NAVIGATIONTOOLS_H
#define NAVIGATIONTOOLS_H

#include <Arduino.h>

class NavigationTools {
public:
    struct FieldPosition {
        float x;
        float y;
    };

    struct VivePosition {
        float x_vive;
        float y_vive;
    };

    //Robot Start Position (Real Coordinates)
    static constexpr float START_X_IN = -75.0f;
    static constexpr float START_Y_IN = 12.0f;

    //Zone 2 Start Position (Real Coordinates)
    static constexpr float ZONE2_WAYPOINT_X = 10.0f;
    static constexpr float ZONE2_WAYPOINT_Y = -6.0f;

    static FieldPosition convertViveToField(VivePosition vivePos) {
        // Your specific coefficients
        const float mx_xr = -0.02326407517;
        const float my_xr = 0.001761464417;
        const float c_xr  = 78.568437;

        const float mx_yr = 0.00132269917;
        const float my_yr = 0.0243494819;
        const float c_yr  = -115.5075201;

        FieldPosition fPos;
        // FIXED: Assign to x then y
        fPos.x = (vivePos.x_vive * mx_xr) + (vivePos.y_vive * my_xr) + c_xr;
        fPos.y = (vivePos.x_vive * mx_yr) + (vivePos.y_vive * my_yr) + c_yr;
        
        return fPos;
    }
};

#endif