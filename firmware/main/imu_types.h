#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* rad/s */
    uint32_t sequence;
} imu_sample_t;

typedef struct {
    float w, x, y, z;
} quat_t;
