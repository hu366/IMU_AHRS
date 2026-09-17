#pragma once

#include <stdbool.h>
#include "imu_types.h"

bool preprocess_sample(const imu_sample_t *raw, imu_sample_t *out);
void preprocess_log_mapping(void);
