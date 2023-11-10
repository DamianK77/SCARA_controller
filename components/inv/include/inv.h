#include <stdio.h>
#include <math.h>
#include "driver/gptimer.h"
#include "esp_log.h"

#define L1  180
#define L2  157
#define D1  0
#define D2  0
#define RAD2DEG 57.295

// Where:
// L1 and L2 are the lengths of the first and second arm segments, respectively.
// D1 is the vertical offset of the robot base from the working plane.
// D2 is the vertical offset of the second joint from the working plane.

void calculate_invkin(float *xyz, float *goal_angs, float *curr_angs, float *delta_angs, float *curr_z, float *delta_z, int *error);
void calculate_fwdkin(float *xyz, float *goal_angs);