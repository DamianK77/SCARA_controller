#include <stdio.h>
#include "inv.h"

// q0 = atan2(y,x) - atan2(L2sin(q1), L1+L2cos(q1))
// q1 = ±acos((x^2 + y^2 - L1^2 - L2^2)/(2L1L2))

// Where:
// L1 and L2 are the lengths of the first and second arm segments, respectively.
// D1 is the vertical offset of the robot base from the working plane.
// D2 is the vertical offset of the second joint from the working plane.

//x = L1cos(q0) + L2cos(q0+q1)
//y = L1sin(q0) + L2sin(q0+q1)


void calculate_invkin(float *xyz, float *goal_angs, float *curr_angs, float *delta_angs)
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

    for (uint8_t i = 0; i < 2; i++)
    {
        delta_angs[i] = goal_angs[i] - curr_angs[i];
    }

    ESP_LOGI("KINEMATICS", "GOAL POS xyz: %f %f %f ==== calculated goal_angs q1 q2: %f %f", xyz[0], xyz[1], xyz[2], goal_angs[0], goal_angs[1]);
    ESP_LOGI("KINEMATICS", "ang deltas: %f %f", delta_angs[0], delta_angs[1]);
}

void calculate_fwdkin(float *xyz, float *goal_angs)
{
    xyz[0] = L1*cos(goal_angs[0]) + L2*cos(goal_angs[0] + goal_angs[1]);
    xyz[1] = L1*sin(goal_angs[0]) + L2*sin(goal_angs[0] + goal_angs[1]);
}