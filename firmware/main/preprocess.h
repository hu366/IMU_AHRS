#pragma once

#include <stdbool.h>
#include "imu_types.h"

bool preprocess_sample(const imu_sample_t *raw, imu_sample_t *out);
void preprocess_log_mapping(void);

/* Gyro bias is hand-frame rad/s, applied after APP_AXIS_* mapping. Not persisted. */
void preprocess_set_gyro_bias(float bx, float by, float bz);
void preprocess_get_gyro_bias(float out[3]);
bool preprocess_has_gyro_bias(void);
