#include "ahrs.h"

#include <math.h>

#include "app_config.h"
#include "esp_log.h"

#if APP_AHRS_ALGO != APP_AHRS_ALGO_MADGWICK && APP_AHRS_ALGO != APP_AHRS_ALGO_VQF
#error "APP_AHRS_ALGO must be APP_AHRS_ALGO_MADGWICK or APP_AHRS_ALGO_VQF"
#endif

#if APP_AHRS_ALGO == APP_AHRS_ALGO_VQF
#include "vqf.h"
#else
#include "madgwick.h"
#endif

static const char *TAG = "ahrs";

#if APP_AHRS_ALGO != APP_AHRS_ALGO_VQF
static madgwick_t s_filter;
#endif
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

/* After normalize, wire |q| must sit in (0.99, 1.01). */
static bool quat_unit_ok(const quat_t *q)
{
    const float n2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    return isfinite(n2) && n2 > (0.99f * 0.99f) && n2 < (1.01f * 1.01f);
}

#if APP_AHRS_ALGO == APP_AHRS_ALGO_VQF
static void init_vqf_filter(void)
{
    const vqf_real_t ts = (vqf_real_t)s_nominal_dt;
    /* This C port ignores magTs<=0 and stores gyrTs. 6D: never call updateMag. */
    initVqf(ts, ts, (vqf_real_t)(-1.0f));
    setMagDistRejectionEnabled(false);
    setMotionBiasEstEnabled(APP_VQF_MOTION_BIAS != 0);
    setRestBiasEstEnabled(APP_VQF_REST_BIAS != 0);
    setTauAcc((vqf_real_t)APP_VQF_TAU_ACC);
}

static void vqf_step(const imu_sample_t *sample, quat_t *q)
{
    const vqf_real_t gyr[3] = {
        (vqf_real_t)sample->gx,
        (vqf_real_t)sample->gy,
        (vqf_real_t)sample->gz,
    };
    const vqf_real_t acc[3] = {
        (vqf_real_t)(sample->ax * APP_GRAVITY_MPS2),
        (vqf_real_t)(sample->ay * APP_GRAVITY_MPS2),
        (vqf_real_t)(sample->az * APP_GRAVITY_MPS2),
    };
    updateGyr(gyr);
    updateAcc(acc);

    vqf_real_t out[4];
    getQuat6D(out);
#if APP_VQF_CONJUGATE_OUTPUT
    q->w = (float)out[0];
    q->x = (float)(-out[1]);
    q->y = (float)(-out[2]);
    q->z = (float)(-out[3]);
#else
    q->w = (float)out[0];
    q->x = (float)out[1];
    q->y = (float)out[2];
    q->z = (float)out[3];
#endif
}
#endif

static void reset_filter(bool warn)
{
#if APP_AHRS_ALGO == APP_AHRS_ALGO_VQF
    /* Full initVqf, not only resetState: a NaN-poisoned filter must not stick. */
    init_vqf_filter();
#else
    madgwick_init(&s_filter, APP_MADGWICK_BETA);
#endif
    s_ready = true;
    if (warn) {
#if APP_AHRS_ALGO == APP_AHRS_ALGO_VQF
        ESP_LOGW(TAG, "filter reset to identity (VQF 6D)");
#else
        ESP_LOGW(TAG, "filter reset to identity, beta=%.3f", APP_MADGWICK_BETA);
#endif
    }
}

void ahrs_init(float sample_period_s)
{
    s_nominal_dt = (sample_period_s > 0.0f) ? sample_period_s : (1.0f / (float)APP_SAMPLE_HZ);
    s_ready = false;
    reset_filter(false);
#if APP_AHRS_ALGO == APP_AHRS_ALGO_VQF
    ESP_LOGI(TAG,
             "init VQF 6D tauAcc=%.3f motion=%d rest=%d mag=off conj=%d dt_nom=%.4f s (fixed Ts)",
             APP_VQF_TAU_ACC, APP_VQF_MOTION_BIAS, APP_VQF_REST_BIAS,
             APP_VQF_CONJUGATE_OUTPUT, s_nominal_dt);
#else
    ESP_LOGI(TAG, "init Madgwick 6DoF beta=%.3f dt_nom=%.4f s",
             APP_MADGWICK_BETA, s_nominal_dt);
#endif
    ESP_LOGI(TAG, "6D AHRS: no magnetometer, yaw around gravity will drift");
}

bool ahrs_update(const imu_sample_t *sample, float dt, quat_t *out)
{
    if (!s_ready) {
        ahrs_init(1.0f / (float)APP_SAMPLE_HZ);
    }
    if (sample == NULL || out == NULL || !isfinite(dt) || dt <= 0.0f) {
        return false;
    }

    quat_t q;
#if APP_AHRS_ALGO == APP_AHRS_ALGO_VQF
    (void)dt; /* vqf-c integrates with coeffs.gyrTs from init, not per-sample dt */
    vqf_step(sample, &q);
#else
    madgwick_update_imu(&s_filter,
                        sample->gx, sample->gy, sample->gz,
                        sample->ax, sample->ay, sample->az,
                        dt);
    q.w = s_filter.q0;
    q.x = s_filter.q1;
    q.y = s_filter.q2;
    q.z = s_filter.q3;
#endif
    if (!quat_ok(&q)) {
        reset_filter(true);
        return false;
    }

    const float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!isfinite(n) || n <= 0.0f) {
        reset_filter(true);
        return false;
    }
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
    if (!quat_unit_ok(&q)) {
        reset_filter(true);
        return false;
    }
    *out = q;
    return true;
}
