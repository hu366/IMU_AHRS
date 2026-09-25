#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void ble_app_start(void);

/* Queue one already-encoded line for BLE TX. Returns false if not connected,
 * not subscribed, or the bounded outbound queue is full. Never blocks the
 * caller waiting for a peer or the NimBLE host. */
bool ble_send_line(const char *line, size_t len);

/* Outbound pose lines dropped because BLE cannot keep up. This is separate
 * from IMU DATA_RDY loss: queued BLE output may be dropped without changing
 * the timestamp semantics of any frame that is sent. */
uint32_t ble_app_tx_queue_drop_count(void);
uint32_t ble_app_tx_failure_count(void);

/* Session identifier used to invalidate PC clock mappings after reboot. */
uint32_t ble_app_boot_id(void);
