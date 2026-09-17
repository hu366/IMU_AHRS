#include "ble_app.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "app_config.h"
#include "ble_uart.h"
#include "pose_protocol.h"

static const char *TAG = "ble_app";

static unsigned s_tx_ok;
static unsigned s_tx_skip;
static unsigned s_tx_fail;
static bool s_logged_skip;

/* MVP RX: keep the NUS RX characteristic, ignore payload. */
static void ble_on_rx(const uint8_t *data, size_t len)
{
    (void)data;
    ESP_LOGI(TAG, "rx %u bytes (ignored)", (unsigned)len);
}

bool ble_send_line(const char *line, size_t len)
{
    if (line == NULL || len == 0) {
        return false;
    }

    const bool connected = ble_uart_is_connected();
    const bool subscribed = ble_uart_is_subscribed();
    if (!connected || !subscribed) {
        s_tx_skip++;
        if (!s_logged_skip) {
            ESP_LOGI(TAG, "skip tx: connected=%d subscribed=%d (will not call tx)",
                     (int)connected, (int)subscribed);
            s_logged_skip = true;
        }
        return false;
    }

    const int rc = ble_uart_tx((const uint8_t *)line, len);
    if (rc != BLE_UART_OK) {
        s_tx_fail++;
        ESP_LOGW(TAG, "tx failed rc=%d len=%u fail=%u",
                 rc, (unsigned)len, s_tx_fail);
        return false;
    }

    if (s_logged_skip) {
        ESP_LOGI(TAG, "tx resumed after %u skipped, fail=%u",
                 s_tx_skip, s_tx_fail);
        s_logged_skip = false;
    }
    s_tx_ok++;
    return true;
}

#if APP_BLE_FAKE_QUAT
static void fake_quat_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / APP_BLE_FAKE_HZ);
    const quat_t q = { .w = 1.f, .x = 0.f, .y = 0.f, .z = 0.f };
    char buf[POSE_PROTOCOL_BUF_LEN];

    ESP_LOGI(TAG, "fake quat ON at %d Hz (unit quaternion). Set APP_BLE_FAKE_QUAT=0 when IMU Notify is live",
             APP_BLE_FAKE_HZ);

    while (1) {
        const int n = pose_protocol_encode(&q, buf, sizeof(buf));
        if (n < 0) {
            ESP_LOGE(TAG, "fake encode failed");
        } else {
            (void)ble_send_line(buf, (size_t)n);
        }
        vTaskDelay(period);
    }
}
#endif

void ble_app_start(void)
{
    ESP_LOGI(TAG, "start name='%s' encrypted=%d fake_quat=%d",
             APP_BLE_DEVICE_NAME, APP_BLE_ENCRYPTED, APP_BLE_FAKE_QUAT);

    if (pose_protocol_self_test() != 0) {
        ESP_LOGE(TAG, "pose_protocol self-test failed; still bringing up BLE");
    }

    const int inst = ble_uart_install(&(ble_uart_config_t){
        .encrypted      = (APP_BLE_ENCRYPTED != 0),
        .device_name    = APP_BLE_DEVICE_NAME,
        .ble_uart_on_rx = ble_on_rx,
    });
    if (inst != BLE_UART_OK) {
        ESP_LOGE(TAG, "ble_uart_install rc=%d (BT not enabled? nvs before NimBLE?)", inst);
        return;
    }

    const int open_rc = ble_uart_open();
    if (open_rc != BLE_UART_OK) {
        ESP_LOGE(TAG, "ble_uart_open rc=%d", open_rc);
        return;
    }
    ESP_LOGI(TAG, "NUS advertising path started (wait for 'advertising as' log)");

#if APP_BLE_FAKE_QUAT
    const BaseType_t ok = xTaskCreate(fake_quat_task, "fake_quat", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create fake_quat task");
    }
#else
    ESP_LOGI(TAG, "fake quat OFF; Agent A imu_task should call ble_send_line");
#endif
}
