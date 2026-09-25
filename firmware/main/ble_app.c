#include "ble_app.h"

#include <string.h>

#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "app_config.h"
#include "ble_uart.h"
#include "pose_protocol.h"
#include "time_sync_protocol.h"

static const char *TAG = "ble_app";

/* BLE transport work must stay outside the DATA_RDY-driven IMU task. */
#define BLE_TX_TASK_STACK       3072
#define BLE_TX_TASK_PRIORITY    3
#define TIME_SYNC_TASK_STACK    3072
#define TIME_SYNC_TASK_PRIORITY 4

typedef struct {
    uint16_t len;
    char line[POSE_PROTOCOL_BUF_LEN];
} ble_tx_line_t;

static volatile uint32_t s_tx_ok;
static volatile uint32_t s_tx_skip;
static volatile uint32_t s_tx_fail;
static volatile uint32_t s_tx_queue_dropped;
static bool s_logged_skip;
static bool s_tx_task_running;
static uint32_t s_boot_id;
static QueueHandle_t s_sync_queue;
static QueueHandle_t s_tx_queue;
static SemaphoreHandle_t s_tx_mutex;
static uint32_t s_sync_rx_ok;
static uint32_t s_sync_rx_invalid;
static uint32_t s_sync_rx_queue_full;

_Static_assert(APP_BLE_TX_BATCH_BYTES >= POSE_PROTOCOL_BUF_LEN,
               "BLE TX batch must fit one complete protocol line");

static bool tx_transport_ready(void)
{
    const bool connected = ble_uart_is_connected();
    const bool subscribed = ble_uart_is_subscribed();
    if (connected && subscribed) {
        return true;
    }

    s_tx_skip++;
    if (!s_logged_skip) {
        ESP_LOGI(TAG, "skip tx: connected=%d subscribed=%d (will not call tx)",
                 (int)connected, (int)subscribed);
        s_logged_skip = true;
    }
    return false;
}

/* Caller owns s_tx_mutex. Serializing line submission prevents a fragmented
 * QT and TSR from interleaving on the NUS byte stream. */
static bool tx_write_locked(const char *line, size_t len)
{
    const int rc = ble_uart_tx((const uint8_t *)line, len);
    if (rc != BLE_UART_OK) {
        s_tx_fail++;
        if (s_tx_fail == 1u || (s_tx_fail % 100u) == 0u) {
            ESP_LOGW(TAG, "tx failed rc=%d len=%u fail=%lu",
                     rc, (unsigned)len, (unsigned long)s_tx_fail);
        }
        return false;
    }

    if (s_logged_skip) {
        ESP_LOGI(TAG, "tx resumed after %lu skipped, fail=%lu",
                 (unsigned long)s_tx_skip, (unsigned long)s_tx_fail);
        s_logged_skip = false;
    }
    s_tx_ok++;
    return true;
}

static bool tx_write_now(const char *line, size_t len)
{
    if (!tx_transport_ready() || s_tx_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_tx_mutex, portMAX_DELAY) != pdTRUE) {
        s_tx_fail++;
        return false;
    }
    const bool sent = tx_write_locked(line, len);
    xSemaphoreGive(s_tx_mutex);
    return sent;
}

static void ble_tx_task(void *arg)
{
    (void)arg;
    static char batch[APP_BLE_TX_BATCH_BYTES];
    ble_tx_line_t item;
    for (;;) {
        if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) != pdPASS) {
            continue;
        }

        size_t batch_len = 0u;
        const TickType_t start_tick = xTaskGetTickCount();
        const TickType_t batch_window = pdMS_TO_TICKS(APP_BLE_TX_BATCH_WAIT_MS);
        for (;;) {
            if (item.len == 0u || item.len > sizeof(item.line)) {
                s_tx_fail++;
            } else {
                if (item.len > sizeof(batch) - batch_len && batch_len > 0u) {
                    (void)tx_write_now(batch, batch_len);
                    batch_len = 0u;
                }
                if (item.len <= sizeof(batch) - batch_len) {
                    memcpy(batch + batch_len, item.line, item.len);
                    batch_len += item.len;
                } else {
                    /* Never emit a partial protocol line. */
                    s_tx_fail++;
                }
            }

            const TickType_t elapsed = xTaskGetTickCount() - start_tick;
            if (elapsed >= batch_window) {
                break;
            }
            const TickType_t remaining = batch_window - elapsed;
            if (xQueueReceive(s_tx_queue, &item, remaining) != pdPASS) {
                break;
            }
        }
        if (batch_len > 0u) {
            (void)tx_write_now(batch, batch_len);
        }
    }
}

static void send_time_sync_response(const time_sync_request_t *request)
{
    if (request == NULL || !tx_transport_ready() || s_tx_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_tx_mutex, portMAX_DELAY) != pdTRUE) {
        s_tx_fail++;
        return;
    }

    /* This is intentionally inside the TX lock and immediately before the
     * stack call, so t3 has the defined TSQ/TSR meaning. */
    const int64_t t3_send_us = esp_timer_get_time();
    char line[APP_TIME_SYNC_LINE_LEN];
    const int n = time_sync_encode_response(s_boot_id, request->sync_id,
                                            request->t2_receive_us, t3_send_us,
                                            line, sizeof(line));
    if (n > 0) {
        (void)tx_write_locked(line, (size_t)n);
    } else {
        s_tx_fail++;
    }
    xSemaphoreGive(s_tx_mutex);
}

static void ble_on_rx(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return;
    }
    uint32_t sync_id = 0;
    const int64_t t2_receive_us = esp_timer_get_time();
    if (time_sync_parse_request(data, len, &sync_id) != 0) {
        s_sync_rx_invalid++;
        if (s_sync_rx_invalid == 1u || (s_sync_rx_invalid % 100u) == 0u) {
            ESP_LOGW(TAG, "reject RX command len=%u invalid=%u",
                     (unsigned)len, s_sync_rx_invalid);
        }
        return;
    }
    if (s_sync_queue == NULL) {
        return;
    }
    const time_sync_request_t request = {
        .sync_id = sync_id,
        .t2_receive_us = t2_receive_us,
    };
    if (xQueueSend(s_sync_queue, &request, 0) != pdPASS) {
        s_sync_rx_queue_full++;
        if (s_sync_rx_queue_full == 1u || (s_sync_rx_queue_full % 100u) == 0u) {
            ESP_LOGW(TAG, "sync request queue full id=%lu full=%u",
                     (unsigned long)sync_id, s_sync_rx_queue_full);
        }
        return;
    }
    s_sync_rx_ok++;
}

static void time_sync_task(void *arg)
{
    (void)arg;
    time_sync_request_t request;
    for (;;) {
        if (xQueueReceive(s_sync_queue, &request, portMAX_DELAY) != pdPASS) {
            continue;
        }
        send_time_sync_response(&request);
    }
}

bool ble_send_line(const char *line, size_t len)
{
    if (line == NULL || len == 0u || len > POSE_PROTOCOL_BUF_LEN
        || !s_tx_task_running || s_tx_queue == NULL) {
        return false;
    }
    if (!tx_transport_ready()) {
        return false;
    }

    ble_tx_line_t item = { .len = (uint16_t)len };
    memcpy(item.line, line, len);
    if (xQueueSend(s_tx_queue, &item, 0) != pdPASS) {
        /* BLE congestion is allowed to lose a transport frame, never the
         * DATA_RDY sample that produced it. */
        s_tx_queue_dropped++;
        return false;
    }
    return true;
}

uint32_t ble_app_tx_queue_drop_count(void)
{
    return s_tx_queue_dropped;
}

uint32_t ble_app_tx_failure_count(void)
{
    return s_tx_fail;
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
    s_boot_id = esp_random();
    if (s_boot_id == 0u) {
        s_boot_id = 1u;
    }

    /* NimBLE emits an INFO line for every notification by default. At 100 Hz
     * that alone can saturate the 115200-baud console and delay IMU work. */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create BLE TX mutex");
    }
    s_tx_queue = xQueueCreate(APP_BLE_TX_QUEUE_LEN, sizeof(ble_tx_line_t));
    if (s_tx_queue == NULL) {
        ESP_LOGE(TAG, "failed to create BLE TX queue len=%d", APP_BLE_TX_QUEUE_LEN);
    }
    s_sync_queue = xQueueCreate(APP_TIME_SYNC_QUEUE_LEN, sizeof(time_sync_request_t));
    if (s_sync_queue == NULL) {
        ESP_LOGE(TAG, "failed to create time sync queue");
    }
    ESP_LOGI(TAG, "start name='%s' encrypted=%d fake_quat=%d",
             APP_BLE_DEVICE_NAME, APP_BLE_ENCRYPTED, APP_BLE_FAKE_QUAT);
    ESP_LOGI(TAG, "boot_id=%lu time sync queue=%d BLE TX queue=%d batch=%dms/%dB",
             (unsigned long)s_boot_id, APP_TIME_SYNC_QUEUE_LEN, APP_BLE_TX_QUEUE_LEN,
             APP_BLE_TX_BATCH_WAIT_MS, APP_BLE_TX_BATCH_BYTES);

    if (pose_protocol_self_test() != 0) {
        ESP_LOGE(TAG, "pose_protocol self-test failed; still bringing up BLE");
    }
    if (time_sync_protocol_self_test() != 0) {
        ESP_LOGE(TAG, "time_sync_protocol self-test failed; still bringing up BLE");
    }

    const int inst = ble_uart_install(&(ble_uart_config_t){
        .encrypted      = (APP_BLE_ENCRYPTED != 0),
        .device_name    = APP_BLE_DEVICE_NAME,
        .ble_uart_on_rx = ble_on_rx,
        .connection_interval_min = APP_BLE_CONN_ITVL_MIN,
        .connection_interval_max = APP_BLE_CONN_ITVL_MAX,
        .connection_latency = APP_BLE_CONN_LATENCY,
        .supervision_timeout = APP_BLE_SUPERVISION_TIMEOUT,
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

    if (s_tx_queue != NULL && s_tx_mutex != NULL) {
        const BaseType_t tx_ok = xTaskCreate(ble_tx_task, "ble_tx", BLE_TX_TASK_STACK,
                                             NULL, BLE_TX_TASK_PRIORITY, NULL);
        if (tx_ok != pdPASS) {
            ESP_LOGE(TAG, "failed to create BLE TX task");
        } else {
            s_tx_task_running = true;
        }
    }

    if (s_sync_queue != NULL && s_tx_mutex != NULL) {
        const BaseType_t ok = xTaskCreate(time_sync_task, "time_sync", TIME_SYNC_TASK_STACK,
                                          NULL, TIME_SYNC_TASK_PRIORITY, NULL);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "failed to create time sync task");
        }
    }

#if APP_BLE_FAKE_QUAT
    const BaseType_t ok = xTaskCreate(fake_quat_task, "fake_quat", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create fake_quat task");
    }
#else
    ESP_LOGI(TAG, "fake quat OFF; Agent A imu_task should call ble_send_line");
#endif
}

uint32_t ble_app_boot_id(void)
{
    return s_boot_id;
}
