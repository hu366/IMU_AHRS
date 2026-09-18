#pragma once

#include <stdbool.h>
#include "imu_types.h"

void ahrs_init(float sample_period_s);
bool ahrs_update(const imu_sample_t *sample, float dt, quat_t *out);

/* [3.2 hook] Display-zero q_hand = conj(q_ref)*q_current would go here.
   Not used: hand frame is APP_AXIS_*; identity is Madgwick +Z up. */
