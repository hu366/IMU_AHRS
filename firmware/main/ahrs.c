#include "ahrs.h"

#include <math.h>

#include "app_config.h"
#include "esp_log.h"
#include "madgwick.h"

static const char *TAG = "ahrs";

static madgwick_t s_filter;
static float s_nominal_dt;
static bool s_ready;

static bool quat_ok(const quat_t *q)
{
    if (!isfinite(q->w) || !isfinite(q->x) || !isfinite(q->y) || !isfinite(q->z)) {
        return false;
    }
    const float n2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    return n2 > 0.25f && n2 < 4.0f;
}

static void reset_filter(bool warn)
{
    madgwick_init(&s_filter, APP_MADGWICK_BETA);
    s_ready = true;
    if (warn) {
        ESP_LOGW(TAG, "filter reset to identity, beta=%.3f", APP_MADGWICK_BETA);
    }
}

void ahrs_init(float sample_period_s)
{
    s_nominal_dt = (sample_period_s > 0.0f) ? sample_period_s : (1.0f / (float)APP_SAMPLE_HZ);
    reset_filter(false);
    ESP_LOGI(TAG, "init Madgwick 6DoF beta=%.3f dt_nom=%.4f s",
             APP_MADGWICK_BETA, s_nominal_dt);
}

bool ahrs_update(const imu_sample_t *sample, float dt, quat_t *out)
{
    if (!s_ready) {
        ahrs_init(1.0f / (float)APP_SAMPLE_HZ);
    }
    if (sample == NULL || out == NULL || !isfinite(dt) || dt <= 0.0f) {
        return false;
    }

    madgwick_update_imu(&s_filter,
                        sample->gx, sample->gy, sample->gz,
                        sample->ax, sample->ay, sample->az,
                        dt);

    quat_t q = {
        .w = s_filter.q0,
        .x = s_filter.q1,
        .y = s_filter.q2,
        .z = s_filter.q3,
    };
    if (!quat_ok(&q)) {
        reset_filter(true);
        return false;
    }

    const float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
    *out = q;
    return true;
}
