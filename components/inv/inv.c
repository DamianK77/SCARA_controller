#include <stdio.h>
#include "inv.h"

// q0 = atan2(y,x) - atan2(L2sin(q1), L1+L2cos(q1))
// q1 = ±acos((x^2 + y^2 - L1^2 - L2^2)/(2L1L2))

// Where:
// L1 and L2 are the lengths of the first and second arm segments, respectively.

//x = L1cos(q0) + L2cos(q0+q1)
//y = L1sin(q0) + L2sin(q0+q1)


void calculate_invkin(float *xyz, float *goal_angs, float *curr_angs, float *delta_angs, float *curr_z, float *delta_z, int *error)
{
    float x = xyz[0];
    float y = xyz[1];
    float z = xyz[2];

    float q0 = 0.0;
    float q1 = 0.0;

    q1 = acos(((x*x)+(y*y)-(L1*L1)-(L2*L2))/(2*L1*L2));
    q0 = atan2(y, x) - atan2((L2*sin(q1)), L1 + L2*cos(q1));

    goal_angs[0] = q0*RAD2DEG;
    goal_angs[1] = q1*RAD2DEG;

    *delta_z = -1*(*curr_z - z);

    *error = 0;
    if (goal_angs[0] > 180 || goal_angs[0] < 0 || goal_angs[1] < -140 || goal_angs[1] > 140 || z < 0 || y < 0 || y > L1+L2 || x > L1+L2 || x < -1*(L1+L2)) {
        *error = 1;
    }

    if (!*error) {
        for (uint8_t i = 0; i < 2; i++)
        {
            delta_angs[i] = -1*(curr_angs[i] - goal_angs[i]);
        }       

        // ESP_LOGI("KINEMATICS", "GOAL POS xyz: %f %f %f ==== calculated goal_angs q1 q2: %f %f goal z: %f", xyz[0], xyz[1], xyz[2], goal_angs[0], goal_angs[1], z);
        // ESP_LOGI("KINEMATICS", "ang deltas: %f %f z delta: %f", delta_angs[0], delta_angs[1], *delta_z);
    } else {
        //ESP_LOGI("KINEMATICS", "ERROR, angle not in range");
        for (uint8_t i = 0; i < 2; i++)
        {
            delta_angs[i] = 0;
        }
    }
}

void calculate_fwdkin(float *xyz, float *goal_angs)
{
    xyz[0] = L1*cos(goal_angs[0]) + L2*cos(goal_angs[0] + goal_angs[1]);
    xyz[1] = L1*sin(goal_angs[0]) + L2*sin(goal_angs[0] + goal_angs[1]);
    //ESP_LOGI("KINEMATICS", "fwkin: %f %f", xyz[0], xyz[1]);
}