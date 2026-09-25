#include "pose_protocol.h"

#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "pose_proto";

static unsigned s_encode_ok;
static unsigned s_encode_fail;

static bool quat_is_finite(const quat_t *q)
{
    return isfinite(q->w) && isfinite(q->x) && isfinite(q->y) && isfinite(q->z);
}

static bool quat_norm_ok(const quat_t *q)
{
    const float n2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    /* 4.3 wire gate: emitted |q| in (0.99, 1.01). Compare n2 to avoid sqrt. */
    return (n2 > 0.99f * 0.99f) && (n2 < 1.01f * 1.01f);
}

int pose_protocol_encode(const quat_t *q, char *buf, size_t buf_len)
{
    if (q == NULL || buf == NULL || buf_len == 0) {
        s_encode_fail++;
        return -1;
    }
    if (!quat_is_finite(q) || !quat_norm_ok(q)) {
        s_encode_fail++;
        ESP_LOGW(TAG, "reject encode fail=%u (nan/inf or bad |q|)", s_encode_fail);
        return -1;
    }

    const int n = snprintf(buf, buf_len, "Q,%.4f,%.4f,%.4f,%.4f\n",
                           q->w, q->x, q->y, q->z);
    if (n <= 0 || (size_t)n >= buf_len || buf[n - 1] != '\n') {
        s_encode_fail++;
        ESP_LOGW(TAG, "encode truncated/no NL n=%d buf_len=%u fail=%u",
                 n, (unsigned)buf_len, s_encode_fail);
        return -1;
    }

    s_encode_ok++;
    return n;
}

int pose_protocol_encode_timed(const quat_t *q,
                               uint32_t boot_id,
                               uint32_t sequence,
                               int64_t t_data_ready_us,
                               char *buf,
                               size_t buf_len)
{
    if (q == NULL || buf == NULL || buf_len == 0) {
        s_encode_fail++;
        return -1;
    }
    if (!quat_is_finite(q) || !quat_norm_ok(q)) {
        s_encode_fail++;
        ESP_LOGW(TAG, "reject timed encode fail=%u (nan/inf or bad |q|)", s_encode_fail);
        return -1;
    }

    const int n = snprintf(buf, buf_len,
                           "QT,%" PRIu32 ",%" PRIu32 ",%" PRId64 ",%.4f,%.4f,%.4f,%.4f\n",
                           boot_id, sequence, t_data_ready_us,
                           q->w, q->x, q->y, q->z);
    if (n <= 0 || (size_t)n >= buf_len || buf[n - 1] != '\n') {
        s_encode_fail++;
        ESP_LOGW(TAG, "timed encode truncated/no NL n=%d buf_len=%u fail=%u",
                 n, (unsigned)buf_len, s_encode_fail);
        return -1;
    }
    s_encode_ok++;
    return n;
}

int pose_protocol_self_test(void)
{
    char buf[POSE_PROTOCOL_BUF_LEN];
    int fails = 0;

    const quat_t q_ident = { .w = 1.f, .x = 0.f, .y = 0.f, .z = 0.f };
    const char *exp_ident = "Q,1.0000,0.0000,0.0000,0.0000\n";
    int n = pose_protocol_encode(&q_ident, buf, sizeof(buf));
    if (n != (int)strlen(exp_ident) || memcmp(buf, exp_ident, (size_t)n) != 0) {
        ESP_LOGE(TAG, "vector ident mismatch n=%d got='%s'", n, n > 0 ? buf : "");
        fails++;
    } else {
        ESP_LOGI(TAG, "vector ident ok (%d bytes)", n);
    }

    const quat_t q_sample = {
        .w = 0.9981f, .x = 0.0123f, .y = -0.0310f, .z = 0.0512f,
    };
    const char *exp_sample = "Q,0.9981,0.0123,-0.0310,0.0512\n";
    n = pose_protocol_encode(&q_sample, buf, sizeof(buf));
    if (n != (int)strlen(exp_sample) || memcmp(buf, exp_sample, (size_t)n) != 0) {
        ESP_LOGE(TAG, "vector sample mismatch n=%d got='%s'", n, n > 0 ? buf : "");
        fails++;
    } else {
        ESP_LOGI(TAG, "vector sample ok (%d bytes)", n);
    }

    const char *exp_timed = "QT,39182744,1523,987654321,0.9981,0.0123,-0.0310,0.0512\n";
    n = pose_protocol_encode_timed(&q_sample, 39182744u, 1523u, 987654321,
                                   buf, sizeof(buf));
    if (n != (int)strlen(exp_timed) || memcmp(buf, exp_timed, (size_t)n) != 0) {
        ESP_LOGE(TAG, "timed vector mismatch n=%d got='%s'", n, n > 0 ? buf : "");
        fails++;
    } else {
        ESP_LOGI(TAG, "timed vector ok (%d bytes)", n);
    }

    n = pose_protocol_encode_timed(&q_ident, 1u, 2u, -3, buf, sizeof(buf));
    if (n <= 0 || strncmp(buf, "QT,1,2,-3,", 10) != 0) {
        ESP_LOGE(TAG, "timed negative timestamp should encode");
        fails++;
    }

    char timed_tiny[16];
    if (pose_protocol_encode_timed(&q_ident, 1u, 2u, 3, timed_tiny,
                                   sizeof(timed_tiny)) != -1) {
        ESP_LOGE(TAG, "timed short buffer should fail");
        fails++;
    }

    char tiny[8];
    if (pose_protocol_encode(&q_ident, tiny, sizeof(tiny)) != -1) {
        ESP_LOGE(TAG, "short buffer should fail");
        fails++;
    }

    quat_t q_nan = q_ident;
    q_nan.w = NAN;
    if (pose_protocol_encode(&q_nan, buf, sizeof(buf)) != -1) {
        ESP_LOGE(TAG, "NaN should fail");
        fails++;
    }

    quat_t q_inf = q_ident;
    q_inf.x = INFINITY;
    if (pose_protocol_encode(&q_inf, buf, sizeof(buf)) != -1) {
        ESP_LOGE(TAG, "Inf should fail");
        fails++;
    }

    quat_t q_zero = { .w = 0.f, .x = 0.f, .y = 0.f, .z = 0.f };
    if (pose_protocol_encode(&q_zero, buf, sizeof(buf)) != -1) {
        ESP_LOGE(TAG, "zero quat should fail");
        fails++;
    }

    quat_t q_long = { .w = 2.f, .x = 0.f, .y = 0.f, .z = 0.f };
    if (pose_protocol_encode(&q_long, buf, sizeof(buf)) != -1) {
        ESP_LOGE(TAG, "|q|=2 should fail");
        fails++;
    }

    quat_t q_short = { .w = 0.5f, .x = 0.f, .y = 0.f, .z = 0.f };
    if (pose_protocol_encode(&q_short, buf, sizeof(buf)) != -1) {
        ESP_LOGE(TAG, "|q|=0.5 should fail");
        fails++;
    }

    if (fails == 0) {
        ESP_LOGI(TAG, "self-test passed");
    } else {
        ESP_LOGE(TAG, "self-test failed (%d)", fails);
    }
    return fails == 0 ? 0 : -1;
}
