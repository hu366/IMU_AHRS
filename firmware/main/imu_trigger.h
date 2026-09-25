#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Configure the DATA_RDY input and bind its notifications to an IMU task. */
esp_err_t imu_trigger_init(gpio_num_t gpio, TaskHandle_t task);

/* Wait for one DATA_RDY event. The ISR puts the timestamp and its source
 * sequence into a queue atomically. If multiple events have accumulated, this
 * returns false after reporting the newest event and the pending count: without
 * an MPU FIFO, their register samples cannot be associated safely. */
bool imu_trigger_wait(int64_t *t_data_ready_us,
                      TickType_t wait_ticks,
                      uint32_t *pending_events,
                      uint32_t *data_ready_sequence);

/* Call once after reading the MPU registers for an event returned by
 * imu_trigger_wait(). A newer DATA_RDY during the read makes the register
 * contents ambiguous, so this returns false and records the dropped event. */
bool imu_trigger_finish_read(uint32_t data_ready_sequence);

/* Record a task-side interval check and expose trigger diagnostics. */
void imu_trigger_note_interval(int64_t interval_us, int64_t expected_us);
uint32_t imu_trigger_missed_data_ready(void);
uint32_t imu_trigger_interval_anomalies(void);
uint32_t imu_trigger_notification_overflows(void);
uint32_t imu_trigger_read_overruns(void);
int64_t imu_trigger_last_time_us(void);
