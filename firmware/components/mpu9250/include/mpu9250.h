#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint32_t i2c_freq_hz;
    uint8_t i2c_addr;
    int accel_fs_g;
    int gyro_fs_dps;
    int sample_hz;
} mpu9250_config_t;

esp_err_t mpu9250_init(const mpu9250_config_t *cfg);
esp_err_t mpu9250_read(float accel_g[3], float gyro_rad_s[3]);
uint8_t mpu9250_whoami(void);
const char *mpu9250_whoami_name(uint8_t id);
bool mpu9250_whoami_allowed(uint8_t id);
uint32_t mpu9250_i2c_error_count(void);

#ifdef __cplusplus
}
#endif
