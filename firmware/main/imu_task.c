#include "imu_task.h"

#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include "ahrs.h"
#include "app_config.h"
#include "ble_app.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_types.h"
#include "mpu9250.h"
#include "pose_protocol.h"
#include "preprocess.h"

static const char *TAG = "imu";

#define IMU_TASK_STACK        8192
#define IMU_TASK_PRIORITY     5
#define IMU_RETRY_DELAY_MS    1000
#define IMU_STATS_PERIOD_S    1.0f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static bool init_sensor(void)
{
    const mpu9250_config_t cfg = {
        .sda_gpio = APP_I2C_SDA_GPIO,
        .scl_gpio = APP_I2C_SCL_GPIO,
        .i2c_freq_hz = APP_I2C_FREQ_HZ,
        .i2c_addr = APP_MPU9250_ADDR,
        .accel_fs_g = APP_ACCEL_FS_G,
        .gyro_fs_dps = APP_GYRO_FS_DPS,
        .sample_hz = APP_SAMPLE_HZ,
    };

    ESP_LOGI(TAG, "I2C SDA=%d SCL=%d addr=0x%02X freq=%d sample=%d Hz accel=±%dg gyro=±%d dps beta=%.3f",
             APP_I2C_SDA_GPIO, APP_I2C_SCL_GPIO, APP_MPU9250_ADDR, APP_I2C_FREQ_HZ,
             APP_SAMPLE_HZ, APP_ACCEL_FS_G, APP_GYRO_FS_DPS, APP_MADGWICK_BETA);
    preprocess_log_mapping();

    esp_err_t err = mpu9250_init(&cfg);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "I2C bus/probe failed (wiring, 3.3V, SDA/SCL, addr 0x%02X)",
                 APP_MPU9250_ADDR);
        return false;
    }
    if (err == ESP_ERR_INVALID_RESPONSE) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch got=0x%02X allowed=0x%02X/0x%02X/0x%02X",
                 mpu9250_whoami(), APP_WHOAMI_MPU9250, APP_WHOAMI_MPU9255, APP_WHOAMI_MPU6500);
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 init failed (%s)", esp_err_to_name(err));
        return false;
    }

    const uint8_t id = mpu9250_whoami();
    ESP_LOGI(TAG, "init ok WHO_AM_I=0x%02X (%s)", id, mpu9250_whoami_name(id));
    return true;
}

static void emit_quat(const quat_t *q)
{
    printf("Q,%.4f,%.4f,%.4f,%.4f\n", q->w, q->x, q->y, q->z);

    char line[POSE_PROTOCOL_BUF_LEN];
    int n = pose_protocol_encode(q, line, sizeof(line));
    if (n > 0) {
        (void)ble_send_line(line, (size_t)n);
    }
}

static void imu_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "imu_task_start");

    bool ready = false;
    while (!ready) {
        ready = init_sensor();
        if (!ready) {
            vTaskDelay(pdMS_TO_TICKS(IMU_RETRY_DELAY_MS));
        }
    }

    const float dt_nom = 1.0f / (float)APP_SAMPLE_HZ;
    ahrs_init(dt_nom);

    uint32_t sequence = 0;
    uint32_t bad_samples = 0;
    uint32_t ahrs_resets = 0;
    uint32_t stats_n = 0;
    float dt_sum = 0.0f;
    float dt_max = 0.0f;
    float acc_sum = 0.0f;
    int64_t t_prev_us = 0;
    int64_t stats_t0_us = esp_timer_get_time();

    TickType_t period_ticks = pdMS_TO_TICKS(1000 / APP_SAMPLE_HZ);
    if (period_ticks < 1) {
        period_ticks = 1;
    }
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period_ticks);

        const int64_t now_us = esp_timer_get_time();
        float dt = dt_nom;
        if (t_prev_us > 0) {
            dt = (float)(now_us - t_prev_us) / 1e6f;
        }
        t_prev_us = now_us;

        const float dt_used = clampf(dt, 0.5f * dt_nom, 2.0f * dt_nom);

        float accel_g[3];
        float gyro_rad[3];
        esp_err_t err = mpu9250_read(accel_g, gyro_rad);
        if (err != ESP_OK) {
            continue;
        }

        sequence++;
        imu_sample_t raw = {
            .ax = accel_g[0],
            .ay = accel_g[1],
            .az = accel_g[2],
            .gx = gyro_rad[0],
            .gy = gyro_rad[1],
            .gz = gyro_rad[2],
            .sequence = sequence,
        };

        imu_sample_t sample;
        if (!preprocess_sample(&raw, &sample)) {
            bad_samples++;
            if (bad_samples == 1u || (bad_samples % 100u) == 0u) {
                ESP_LOGW(TAG, "drop illegal sample seq=%lu ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f",
                         (unsigned long)sequence, raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz);
            }
            continue;
        }

        quat_t q;
        if (!ahrs_update(&sample, dt_used, &q)) {
            ahrs_resets++;
            ESP_LOGW(TAG, "ahrs update failed/reset count=%lu seq=%lu dt=%.4f",
                     (unsigned long)ahrs_resets, (unsigned long)sequence, dt_used);
            continue;
        }

        emit_quat(&q);

        const float acc_mag = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
        const float qn = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        stats_n++;
        dt_sum += dt;
        if (dt > dt_max) {
            dt_max = dt;
        }
        acc_sum += acc_mag;

        const float stats_age = (float)(now_us - stats_t0_us) / 1e6f;
        if (stats_age >= IMU_STATS_PERIOD_S) {
            const float dt_mean = (stats_n > 0) ? (dt_sum / (float)stats_n) : 0.0f;
            const float acc_mean = (stats_n > 0) ? (acc_sum / (float)stats_n) : 0.0f;
            ESP_LOGI(TAG,
                     "stats n=%lu |q|=%.4f |a|=%.3fg dt_mean=%.4fs dt_max=%.4fs seq=%lu i2c_err=%lu bad=%lu ahrs_reset=%lu",
                     (unsigned long)stats_n, qn, acc_mean, dt_mean, dt_max,
                     (unsigned long)sequence,
                     (unsigned long)mpu9250_i2c_error_count(),
                     (unsigned long)bad_samples,
                     (unsigned long)ahrs_resets);
            stats_n = 0;
            dt_sum = 0.0f;
            dt_max = 0.0f;
            acc_sum = 0.0f;
            stats_t0_us = now_us;
        }
    }
}

void imu_task_start(void)
{
    ESP_LOGI(TAG, "creating sample task %d Hz", APP_SAMPLE_HZ);
    BaseType_t ok = xTaskCreate(imu_task, "imu", IMU_TASK_STACK, NULL,
                                IMU_TASK_PRIORITY, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed — IMU path not running");
    }
}
