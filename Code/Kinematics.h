#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <math.h>

// These variables are defined in encoders.h
extern volatile long count_e0;
extern volatile long count_e1;

// ======================================================
// Robot physical parameters (calibrate for accuracy)
// ======================================================
const float count_per_rev = 358.3;   // encoder counts per wheel revolution
const float wheel_radius  = 16.3;    // mm
const float wheel_sep     = 42.7;    // mm (wheel-to-wheel separation)

// Distance per encoder count (mm/count)
const float mm_per_count =
    (2.0 * wheel_radius * PI) / count_per_rev;

// ======================================================
// KINEMATICS CLASS
// ======================================================
class Kinematics_c {

public:

    // Robot pose (global frame)
    float x;
    float y;
    float theta;

    // Previous encoder values
    long last_e1;
    long last_e0;

    // Constructor
    Kinematics_c() {}

    // Initialise pose and encoder reference
    void initialise(float start_x, float start_y, float start_th) {
        last_e0 = count_e0;
        last_e1 = count_e1;

        x = start_x;
        y = start_y;
        theta = start_th;
    }

    // ==================================================
    // UPDATE KINEMATICS
    // ==================================================
    void update() {

        long delta_e0;
        long delta_e1;

        float mean_delta;

        float x_contribution;
        float th_contribution;

        // Encoder delta since last update
        delta_e1 = count_e1 - last_e1;
        delta_e0 = count_e0 - last_e0;

        // Store current counts for next update
        last_e1 = count_e1;
        last_e0 = count_e0;

        // Linear movement (forward/backward)
        mean_delta = (float)delta_e1 + (float)delta_e0;
        mean_delta /= 2.0;

        x_contribution = mean_delta * mm_per_count;

        // Rotation contribution
        th_contribution =
            ((float)delta_e0 - (float)delta_e1)
            * mm_per_count
            / (wheel_sep * 2.0);

        // Convert local motion to global frame
        x += x_contribution * cos(theta);
        y += x_contribution * sin(theta);
        theta += th_contribution;
    }
};

#endif
