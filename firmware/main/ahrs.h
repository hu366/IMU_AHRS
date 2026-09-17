#pragma once

#include <stdbool.h>
#include "imu_types.h"

void ahrs_init(float sample_period_s);
bool ahrs_update(const imu_sample_t *sample, float dt, quat_t *out);
