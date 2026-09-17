#include "mpu9250.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu9250";

#define MPU9250_REG_SMPLRT_DIV     0x19
#define MPU9250_REG_CONFIG         0x1A
#define MPU9250_REG_GYRO_CONFIG    0x1B
#define MPU9250_REG_ACCEL_CONFIG   0x1C
#define MPU9250_REG_ACCEL_CONFIG2  0x1D
#define MPU9250_REG_ACCEL_XOUT_H   0x3B
#define MPU9250_REG_USER_CTRL      0x6A
#define MPU9250_REG_PWR_MGMT_1     0x6B
#define MPU9250_REG_PWR_MGMT_2     0x6C
#define MPU9250_REG_WHO_AM_I       0x75

#define MPU9250_RESET_BIT          7
#define MPU9250_I2C_TIMEOUT_MS     50
#define MPU9250_BURST_LEN          14

#define WHOAMI_MPU9250             0x71
#define WHOAMI_MPU9255             0x73
#define WHOAMI_MPU6500             0x70

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    float accel_scale;
    float gyro_scale;
    uint32_t i2c_errors;
    uint8_t whoami;
    bool ready;
    bool bus_owned;
} mpu9250_ctx_t;

static mpu9250_ctx_t s_ctx;

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint8_t accel_afs_sel(int fs_g)
{
    switch (fs_g) {
    case 16:
        return 3;
    case 8:
        return 2;
    case 4:
        return 1;
    case 2:
    default:
        return 0;
    }
}

static uint8_t gyro_fs_sel(int fs_dps)
{
    switch (fs_dps) {
    case 2000:
        return 3;
    case 1000:
        return 2;
    case 500:
        return 1;
    case 250:
    default:
        return 0;
    }
}

static int accel_fs_from_sel(uint8_t sel)
{
    static const int k_fs[] = {2, 4, 8, 16};
    return k_fs[sel & 0x03];
}

static int gyro_fs_from_sel(uint8_t sel)
{
    static const int k_fs[] = {250, 500, 1000, 2000};
    return k_fs[sel & 0x03];
}

static void note_i2c_error(esp_err_t err, const char *what)
{
    s_ctx.i2c_errors++;
    if (s_ctx.i2c_errors == 1u || (s_ctx.i2c_errors % 100u) == 0u) {
        ESP_LOGE(TAG, "%s failed (%s) i2c_err=%lu",
                 what, esp_err_to_name(err), (unsigned long)s_ctx.i2c_errors);
    }
}

static esp_err_t reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_transmit_receive(s_ctx.dev, &reg, 1, data, len,
                                                MPU9250_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        note_i2c_error(err, "read");
    }
    return err;
}

static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(s_ctx.dev, buf, sizeof(buf),
                                        MPU9250_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        note_i2c_error(err, "write");
    }
    return err;
}

static void teardown_bus(void)
{
    if (s_ctx.dev) {
        i2c_master_bus_rm_device(s_ctx.dev);
        s_ctx.dev = NULL;
    }
    if (s_ctx.bus_owned && s_ctx.bus) {
        i2c_del_master_bus(s_ctx.bus);
        s_ctx.bus = NULL;
        s_ctx.bus_owned = false;
    }
    s_ctx.ready = false;
}

bool mpu9250_whoami_allowed(uint8_t id)
{
    return id == WHOAMI_MPU9250
        || id == WHOAMI_MPU9255
        || id == WHOAMI_MPU6500;
}

const char *mpu9250_whoami_name(uint8_t id)
{
    switch (id) {
    case WHOAMI_MPU9250:
        return "MPU9250";
    case WHOAMI_MPU9255:
        return "MPU9255";
    case WHOAMI_MPU6500:
        return "MPU6500";
    default:
        return "unknown";
    }
}

uint8_t mpu9250_whoami(void)
{
    return s_ctx.whoami;
}

uint32_t mpu9250_i2c_error_count(void)
{
    return s_ctx.i2c_errors;
}

esp_err_t mpu9250_init(const mpu9250_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    teardown_bus();
    memset(&s_ctx, 0, sizeof(s_ctx));

    const uint8_t accel_sel = accel_afs_sel(cfg->accel_fs_g);
    const uint8_t gyro_sel = gyro_fs_sel(cfg->gyro_fs_dps);
    const int accel_fs = accel_fs_from_sel(accel_sel);
    const int gyro_fs = gyro_fs_from_sel(gyro_sel);
    int sample_hz = cfg->sample_hz > 0 ? cfg->sample_hz : 100;
    if (sample_hz > 1000) {
        sample_hz = 1000;
    }
    const uint8_t smplrt_div = (uint8_t)((1000 / sample_hz) - 1);

    s_ctx.accel_scale = (float)accel_fs / 32768.0f;
    s_ctx.gyro_scale = ((float)gyro_fs / 32768.0f) * ((float)M_PI / 180.0f);

    ESP_LOGI(TAG, "init SDA=%d SCL=%d addr=0x%02X freq=%lu accel=±%dg gyro=±%d dps sample=%d Hz",
             cfg->sda_gpio, cfg->scl_gpio, cfg->i2c_addr,
             (unsigned long)cfg->i2c_freq_hz, accel_fs, gyro_fs, sample_hz);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)cfg->sda_gpio,
        .scl_io_num = (gpio_num_t)cfg->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &s_ctx.bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus create failed (%s)", esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    s_ctx.bus_owned = true;

    err = i2c_master_probe(s_ctx.bus, cfg->i2c_addr, MPU9250_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C probe failed addr=0x%02X (%s) — check VCC/GND/SDA/SCL/pullup",
                 cfg->i2c_addr, esp_err_to_name(err));
        unsigned found = 0;
        for (uint8_t a = 0x08; a < 0x78; a++) {
            if (i2c_master_probe(s_ctx.bus, a, 20) == ESP_OK) {
                ESP_LOGW(TAG, "I2C scan found 0x%02X%s", a,
                         (a == 0x69) ? " (MPU AD0=VCC — set APP_MPU9250_ADDR 0x69)" : "");
                found++;
            }
        }
        if (found == 0) {
            ESP_LOGE(TAG, "I2C scan: no device 0x08-0x77. Swap SDA/SCL, check 3.3V/GND, confirm GPIO6/7 silkscreen");
        }
        teardown_bus();
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->i2c_addr,
        .scl_speed_hz = cfg->i2c_freq_hz,
    };
    err = i2c_master_bus_add_device(s_ctx.bus, &dev_config, &s_ctx.dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add device failed (%s)", esp_err_to_name(err));
        teardown_bus();
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t whoami = 0;
    err = reg_read(MPU9250_REG_WHO_AM_I, &whoami, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed (%s)", esp_err_to_name(err));
        teardown_bus();
        return ESP_ERR_NOT_FOUND;
    }
    s_ctx.whoami = whoami;
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X (%s)", whoami, mpu9250_whoami_name(whoami));
    if (!mpu9250_whoami_allowed(whoami)) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, allowed 0x%02X/0x%02X/0x%02X",
                 whoami, WHOAMI_MPU9250, WHOAMI_MPU9255, WHOAMI_MPU6500);
        teardown_bus();
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = reg_write(MPU9250_REG_PWR_MGMT_1, (uint8_t)(1u << MPU9250_RESET_BIT));
    if (err != ESP_OK) {
        teardown_bus();
        return ESP_ERR_NOT_FOUND;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t whoami_after = 0;
    err = reg_read(MPU9250_REG_WHO_AM_I, &whoami_after, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I after reset failed (%s)", esp_err_to_name(err));
        teardown_bus();
        return ESP_ERR_NOT_FOUND;
    }
    if (whoami_after != whoami) {
        ESP_LOGE(TAG, "WHO_AM_I not stable: before=0x%02X after=0x%02X", whoami, whoami_after);
        teardown_bus();
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "WHO_AM_I stable after reset: 0x%02X", whoami_after);

    /* CLKSEL=1: auto/PLL. Clears SLEEP after reset. */
    err = reg_write(MPU9250_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        teardown_bus();
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    const uint8_t writes[][2] = {
        {MPU9250_REG_PWR_MGMT_2, 0x00},
        {MPU9250_REG_USER_CTRL, 0x00},
        {MPU9250_REG_CONFIG, 0x03},          /* gyro DLPF ~41 Hz, 1 kHz */
        {MPU9250_REG_GYRO_CONFIG, (uint8_t)(gyro_sel << 3)},
        {MPU9250_REG_ACCEL_CONFIG, (uint8_t)(accel_sel << 3)},
        {MPU9250_REG_ACCEL_CONFIG2, 0x03},   /* accel DLPF ~41 Hz */
        {MPU9250_REG_SMPLRT_DIV, smplrt_div},
    };
    for (size_t i = 0; i < sizeof(writes) / sizeof(writes[0]); i++) {
        err = reg_write(writes[i][0], writes[i][1]);
        if (err != ESP_OK) {
            teardown_bus();
            return err;
        }
    }

    s_ctx.whoami = whoami_after;
    s_ctx.ready = true;
    ESP_LOGI(TAG, "configured SMPLRT_DIV=%u DLPF=41Hz", smplrt_div);
    return ESP_OK;
}

esp_err_t mpu9250_read(float accel_g[3], float gyro_rad_s[3])
{
    if (!s_ctx.ready || accel_g == NULL || gyro_rad_s == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[MPU9250_BURST_LEN];
    esp_err_t err = reg_read(MPU9250_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    accel_g[0] = (float)be16(&raw[0]) * s_ctx.accel_scale;
    accel_g[1] = (float)be16(&raw[2]) * s_ctx.accel_scale;
    accel_g[2] = (float)be16(&raw[4]) * s_ctx.accel_scale;
    gyro_rad_s[0] = (float)be16(&raw[8]) * s_ctx.gyro_scale;
    gyro_rad_s[1] = (float)be16(&raw[10]) * s_ctx.gyro_scale;
    gyro_rad_s[2] = (float)be16(&raw[12]) * s_ctx.gyro_scale;
    return ESP_OK;
}
