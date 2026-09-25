#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t sync_id;
    int64_t t2_receive_us;
} time_sync_request_t;

/* Parse one complete TSQ line (with or without the trailing newline). */
int time_sync_parse_request(const uint8_t *data, size_t len, uint32_t *sync_id);

/* Encode one complete TSR line. Returns bytes including newline, or -1. */
int time_sync_encode_response(uint32_t boot_id,
                              uint32_t sync_id,
                              int64_t t2_receive_us,
                              int64_t t3_send_us,
                              char *buf,
                              size_t buf_len);

int time_sync_protocol_self_test(void);
