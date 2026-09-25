#include "imu_trigger.h"

#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"

static const char *TAG = "imu_trigger";

typedef struct {
    int64_t t_data_ready_us;
    uint32_t sequence;
} data_ready_event_t;

static TaskHandle_t s_task;
static gpio_num_t s_gpio = (gpio_num_t)-1;
static QueueHandle_t s_event_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t s_last_time_us;
static volatile uint32_t s_missed_data_ready;
static volatile uint32_t s_interval_anomalies;
static volatile uint32_t s_notification_overflows;
static volatile uint32_t s_read_overruns;
static uint32_t s_data_ready_sequence;
static bool s_initialized;

static void IRAM_ATTR data_ready_isr(void *arg)
{
    (void)arg;
    const int64_t now_us = esp_timer_get_time();
    BaseType_t higher_priority_task_woken = pdFALSE;
    data_ready_event_t event = {
        .t_data_ready_us = now_us,
        .sequence = 0,
    };

    portENTER_CRITICAL_ISR(&s_lock);
    s_last_time_us = now_us;
    event.sequence = ++s_data_ready_sequence;
    const bool can_queue = s_task != NULL && s_event_queue != NULL;
    portEXIT_CRITICAL_ISR(&s_lock);

    if (!can_queue || xQueueSendFromISR(s_event_queue, &event,
                                        &higher_priority_task_woken) != pdPASS) {
        portENTER_CRITICAL_ISR(&s_lock);
        s_missed_data_ready++;
        s_notification_overflows++;
        portEXIT_CRITICAL_ISR(&s_lock);
    }

    if (higher_priority_task_woken != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t imu_trigger_init(gpio_num_t gpio, TaskHandle_t task)
{
    if (task == NULL || gpio < 0 || gpio >= GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized) {
        return (s_gpio == gpio && s_task == task) ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    QueueHandle_t event_queue =
        xQueueCreate(APP_IMU_TRIGGER_QUEUE_LEN, sizeof(data_ready_event_t));
    if (event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t config = {
        .pin_bit_mask = 1ULL << (unsigned)gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
#if APP_MPU9250_INT_ACTIVE_HIGH
        .intr_type = GPIO_INTR_POSEDGE,
#else
        .intr_type = GPIO_INTR_NEGEDGE,
#endif
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        vQueueDelete(event_queue);
        return err;
    }

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio ISR service install failed: %s", esp_err_to_name(err));
        vQueueDelete(event_queue);
        return err;
    }

    portENTER_CRITICAL(&s_lock);
    s_gpio = gpio;
    s_task = task;
    s_event_queue = event_queue;
    s_last_time_us = 0;
    s_missed_data_ready = 0;
    s_interval_anomalies = 0;
    s_notification_overflows = 0;
    s_read_overruns = 0;
    s_data_ready_sequence = 0;
    portEXIT_CRITICAL(&s_lock);

    err = gpio_isr_handler_add(gpio, data_ready_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d ISR registration failed: %s", (int)gpio, esp_err_to_name(err));
        portENTER_CRITICAL(&s_lock);
        s_gpio = (gpio_num_t)-1;
        s_task = NULL;
        s_event_queue = NULL;
        portEXIT_CRITICAL(&s_lock);
        vQueueDelete(event_queue);
        return err;
    }

    portENTER_CRITICAL(&s_lock);
    s_initialized = true;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "DATA_RDY GPIO%d configured (%s edge, 3.3V logic)",
             (int)gpio,
#if APP_MPU9250_INT_ACTIVE_HIGH
             "rising"
#else
             "falling"
#endif
    );
    return ESP_OK;
}

bool imu_trigger_wait(int64_t *t_data_ready_us,
                      TickType_t wait_ticks,
                      uint32_t *pending_events,
                      uint32_t *data_ready_sequence)
{
    if (!s_initialized || s_task == NULL || s_event_queue == NULL
        || t_data_ready_us == NULL || data_ready_sequence == NULL) {
        return false;
    }

    data_ready_event_t event;
    if (xQueueReceive(s_event_queue, &event, wait_ticks) != pdPASS) {
        if (pending_events != NULL) {
            *pending_events = 0;
        }
        return false;
    }

    uint32_t count = 1;
    data_ready_event_t pending;
    while (xQueueReceive(s_event_queue, &pending, 0) == pdPASS) {
        event = pending;
        count++;
    }
    if (pending_events != NULL) {
        *pending_events = count;
    }
    *t_data_ready_us = event.t_data_ready_us;
    *data_ready_sequence = event.sequence;

    portENTER_CRITICAL(&s_lock);
    if (count > 1u) {
        s_missed_data_ready += count;
    }
    portEXIT_CRITICAL(&s_lock);

    if (count > 1u || event.t_data_ready_us <= 0) {
        return false;
    }
    return true;
}

bool imu_trigger_finish_read(uint32_t data_ready_sequence)
{
    bool valid;
    portENTER_CRITICAL(&s_lock);
    valid = s_data_ready_sequence == data_ready_sequence;
    if (!valid) {
        /* A newer DATA_RDY may have replaced the register values while I2C
         * was in flight. Drop this event rather than emit a false timestamp. */
        s_missed_data_ready++;
        s_read_overruns++;
    }
    portEXIT_CRITICAL(&s_lock);
    return valid;
}

void imu_trigger_note_interval(int64_t interval_us, int64_t expected_us)
{
    if (interval_us <= 0 || expected_us <= 0) {
        portENTER_CRITICAL(&s_lock);
        s_interval_anomalies++;
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    /* A 50% window catches missed/duplicated 100 Hz events without treating
     * ordinary ISR scheduling jitter as a lost sample. */
    if (interval_us < expected_us / 2 || interval_us > expected_us * 3 / 2) {
        portENTER_CRITICAL(&s_lock);
        s_interval_anomalies++;
        portEXIT_CRITICAL(&s_lock);
    }
}

uint32_t imu_trigger_missed_data_ready(void)
{
    uint32_t value;
    portENTER_CRITICAL(&s_lock);
    value = s_missed_data_ready;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

uint32_t imu_trigger_interval_anomalies(void)
{
    uint32_t value;
    portENTER_CRITICAL(&s_lock);
    value = s_interval_anomalies;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

uint32_t imu_trigger_notification_overflows(void)
{
    uint32_t value;
    portENTER_CRITICAL(&s_lock);
    value = s_notification_overflows;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

uint32_t imu_trigger_read_overruns(void)
{
    uint32_t value;
    portENTER_CRITICAL(&s_lock);
    value = s_read_overruns;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

int64_t imu_trigger_last_time_us(void)
{
    int64_t value;
    portENTER_CRITICAL(&s_lock);
    value = s_last_time_us;
    portEXIT_CRITICAL(&s_lock);
    return value;
}
