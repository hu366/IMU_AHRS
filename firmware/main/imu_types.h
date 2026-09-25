#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* rad/s */
    uint32_t sequence;
    int64_t t_data_ready_us; /* ESP32 esp_timer time captured at DATA_RDY */
} imu_sample_t;

typedef struct {
    float w, x, y, z;
} quat_t;
