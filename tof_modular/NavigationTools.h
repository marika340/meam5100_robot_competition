#ifndef NAVIGATIONTOOLS_H
#define NAVIGATIONTOOLS_H

#include <Arduino.h>
#include <math.h>

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

    // ---------------------------------------------------------------
    // Calibration coefficients (updatable at runtime via calibration UI)
    // x_real = mx_xr*Xvive + my_xr*Yvive + c_xr
    // y_real = mx_yr*Xvive + my_yr*Yvive + c_yr
    // Default values from the center-field LINEST calibration.
    // ---------------------------------------------------------------
    inline static float mx_xr = -0.02326407517f;
    inline static float my_xr =  0.001761464417f;
    inline static float c_xr  =  78.568437f;

    inline static float mx_yr =  0.00132269917f;
    inline static float my_yr =  0.0243494819f;
    inline static float c_yr  = -115.5075201f;

    // Update all six coefficients at once (called by calibration web handler).
    static void updateCalibration(float new_mx_xr, float new_my_xr, float new_c_xr,
                                   float new_mx_yr, float new_my_yr, float new_c_yr) {
        mx_xr = new_mx_xr;
        my_xr = new_my_xr;
        c_xr  = new_c_xr;
        mx_yr = new_mx_yr;
        my_yr = new_my_yr;
        c_yr  = new_c_yr;
    }

    static FieldPosition convertViveToField(VivePosition vivePos) {
        FieldPosition fPos;
        fPos.x = (vivePos.x_vive * mx_xr) + (vivePos.y_vive * my_xr) + c_xr;
        fPos.y = (vivePos.x_vive * mx_yr) + (vivePos.y_vive * my_yr) + c_yr;
        return fPos;
    }

    // ---------------------------------------------------------------
    // runViveRegression -- LINEST-equivalent multiple linear regression.
    //
    // Given n measured Vive points (vx[], vy[]) and their known real
    // coordinates (real_x[], real_y[]), fits:
    //   x_real = a*Xv + b*Yv + c
    //   y_real = d*Xv + e*Yv + f
    // via ordinary least squares (normal equations, Gaussian elimination).
    //
    // Writes results directly into NavigationTools coefficients.
    // Returns true on success, false if the design matrix is singular.
    // ---------------------------------------------------------------
    static bool runViveRegression(const float* vx, const float* vy, int n,
                                   const float* real_x, const float* real_y) {
        // Build A^T A (3x3) and A^T b_x, A^T b_y (3x1).
        // Design matrix A: row i = [vx[i], vy[i], 1]
        double AtA[3][3] = {};
        double AtBx[3]   = {};
        double AtBy[3]   = {};

        for (int i = 0; i < n; i++) {
            double row[3] = { (double)vx[i], (double)vy[i], 1.0 };
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) AtA[r][c] += row[r] * row[c];
                AtBx[r] += row[r] * (double)real_x[i];
                AtBy[r] += row[r] * (double)real_y[i];
            }
        }

        double bx[3], by_[3];
        if (!solve3x3(AtA, AtBx, bx))  return false;
        if (!solve3x3(AtA, AtBy, by_)) return false;

        // bx = [a, b, c]  =>  x_real = a*Xv + b*Yv + c
        // by_= [d, e, f]  =>  y_real = d*Xv + e*Yv + f
        updateCalibration((float)bx[0],  (float)bx[1],  (float)bx[2],
                          (float)by_[0], (float)by_[1], (float)by_[2]);
        return true;
    }

private:
    // Solve a 3x3 linear system Ax=b using Gauss-Jordan elimination
    // with partial pivoting.  Returns false if the matrix is singular.
    static bool solve3x3(double A[3][3], double b[3], double x[3]) {
        // Build augmented matrix [A | b]
        double aug[3][4];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) aug[i][j] = A[i][j];
            aug[i][3] = b[i];
        }

        for (int col = 0; col < 3; col++) {
            // Partial pivot: find the row with the largest absolute value
            int pivot = col;
            for (int row = col + 1; row < 3; row++) {
                if (fabs(aug[row][col]) > fabs(aug[pivot][col])) pivot = row;
            }
            // Swap pivot row into the diagonal position
            if (pivot != col) {
                for (int j = 0; j < 4; j++) {
                    double tmp    = aug[col][j];
                    aug[col][j]   = aug[pivot][j];
                    aug[pivot][j] = tmp;
                }
            }
            if (fabs(aug[col][col]) < 1e-12) return false; // singular

            // Eliminate all other rows in this column
            for (int row = 0; row < 3; row++) {
                if (row == col) continue;
                double factor = aug[row][col] / aug[col][col];
                for (int j = col; j < 4; j++) aug[row][j] -= factor * aug[col][j];
            }
        }

        for (int i = 0; i < 3; i++) x[i] = aug[i][3] / aug[i][i];
        return true;
    }
};

#endif
