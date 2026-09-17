#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "ble_app.h"
#include "imu_task.h"

void app_main(void)
{
    ESP_LOGI("app", "IMU-AHRS start");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ble_app_start();
    imu_task_start();
}
