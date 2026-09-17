#pragma once

#include <stdbool.h>
#include <stddef.h>

void ble_app_start(void);

/* Send one already-encoded line. Returns false if not connected,
 * not subscribed, or tx fails. Never blocks waiting for a peer. */
bool ble_send_line(const char *line, size_t len);
