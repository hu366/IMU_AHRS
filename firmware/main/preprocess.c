#include "preprocess.h"

#include <math.h>

#include "app_config.h"
#include "esp_log.h"

static const char *TAG = "imu";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Sensor axis -> hand frame. Only this file (and app_config.h) may define mapping. */
static const int k_permute[3] = {
    APP_AXIS_PERMUTE_X,
    APP_AXIS_PERMUTE_Y,
    APP_AXIS_PERMUTE_Z,
};
static const int k_sign[3] = {
    APP_AXIS_SIGN_X,
    APP_AXIS_SIGN_Y,
    APP_AXIS_SIGN_Z,
};

static const float k_accel_limit_g = (float)APP_ACCEL_FS_G * 1.5f;
static const float k_gyro_limit_rad =
    (float)APP_GYRO_FS_DPS * 1.5f * ((float)M_PI / 180.0f);

static bool axis_map_valid(void)
{
    for (int i = 0; i < 3; i++) {
        if (k_permute[i] < 0 || k_permute[i] > 2) {
            return false;
        }
        if (k_sign[i] != 1 && k_sign[i] != -1) {
            return false;
        }
    }
    return true;
}

static bool finite6(const imu_sample_t *s)
{
    return isfinite(s->ax) && isfinite(s->ay) && isfinite(s->az)
        && isfinite(s->gx) && isfinite(s->gy) && isfinite(s->gz);
}

static bool in_range(const imu_sample_t *s)
{
    if (fabsf(s->ax) > k_accel_limit_g || fabsf(s->ay) > k_accel_limit_g
        || fabsf(s->az) > k_accel_limit_g) {
        return false;
    }
    if (fabsf(s->gx) > k_gyro_limit_rad || fabsf(s->gy) > k_gyro_limit_rad
        || fabsf(s->gz) > k_gyro_limit_rad) {
        return false;
    }
    return true;
}

void preprocess_log_mapping(void)
{
    ESP_LOGI(TAG, "axis permute=[%d,%d,%d] sign=[%d,%d,%d] (sensor -> hand)",
             k_permute[0], k_permute[1], k_permute[2],
             k_sign[0], k_sign[1], k_sign[2]);
    if (!axis_map_valid()) {
        ESP_LOGE(TAG, "APP_AXIS_* invalid; samples will be dropped");
    }
}

bool preprocess_sample(const imu_sample_t *raw, imu_sample_t *out)
{
    if (raw == NULL || out == NULL || !axis_map_valid() || !finite6(raw) || !in_range(raw)) {
        return false;
    }

    const float acc_s[3] = {raw->ax, raw->ay, raw->az};
    const float gyr_s[3] = {raw->gx, raw->gy, raw->gz};

    imu_sample_t mapped = {
        .ax = (float)k_sign[0] * acc_s[k_permute[0]],
        .ay = (float)k_sign[1] * acc_s[k_permute[1]],
        .az = (float)k_sign[2] * acc_s[k_permute[2]],
        .gx = (float)k_sign[0] * gyr_s[k_permute[0]],
        .gy = (float)k_sign[1] * gyr_s[k_permute[1]],
        .gz = (float)k_sign[2] * gyr_s[k_permute[2]],
        .sequence = raw->sequence,
    };

    if (!finite6(&mapped) || !in_range(&mapped)) {
        return false;
    }

    *out = mapped;
    return true;
}
