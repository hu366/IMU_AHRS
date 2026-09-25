#include "time_sync_protocol.h"

#include <ctype.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "time_sync_proto";

static int parse_u32_field(const char *text, uint32_t *out)
{
    if (text == NULL || out == NULL || *text == '\0') {
        return -1;
    }
    uint64_t value = 0;
    for (const unsigned char *p = (const unsigned char *)text; *p != '\0'; p++) {
        if (!isdigit(*p)) {
            return -1;
        }
        value = value * 10u + (uint64_t)(*p - '0');
        if (value > UINT32_MAX) {
            return -1;
        }
    }
    *out = (uint32_t)value;
    return 0;
}

int time_sync_parse_request(const uint8_t *data, size_t len, uint32_t *sync_id)
{
    if (data == NULL || sync_id == NULL || len == 0 || len >= 64) {
        return -1;
    }

    char line[64];
    memcpy(line, data, len);
    line[len] = '\0';
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'
                       || line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[--len] = '\0';
    }
    if (strncmp(line, "TSQ,", 4) != 0) {
        return -1;
    }
    const char *field = line + 4;
    if (*field == '\0' || strchr(field, ',') != NULL) {
        return -1;
    }
    return parse_u32_field(field, sync_id);
}

int time_sync_encode_response(uint32_t boot_id,
                              uint32_t sync_id,
                              int64_t t2_receive_us,
                              int64_t t3_send_us,
                              char *buf,
                              size_t buf_len)
{
    if (buf == NULL || buf_len == 0 || t2_receive_us < 0 || t3_send_us < t2_receive_us) {
        return -1;
    }
    const int n = snprintf(buf, buf_len,
                           "TSR,%" PRIu32 ",%" PRIu32 ",%" PRId64 ",%" PRId64 "\n",
                           boot_id, sync_id, t2_receive_us, t3_send_us);
    if (n <= 0 || (size_t)n >= buf_len || buf[n - 1] != '\n') {
        return -1;
    }
    return n;
}

int time_sync_protocol_self_test(void)
{
    int fails = 0;
    uint32_t id = 0;
    if (time_sync_parse_request((const uint8_t *)"TSQ,42\n", 7, &id) != 0 || id != 42u) {
        fails++;
    }
    if (time_sync_parse_request((const uint8_t *)"TSQ,-1\n", 7, &id) == 0) {
        fails++;
    }
    if (time_sync_parse_request((const uint8_t *)"TSQ,1,2\n", 8, &id) == 0) {
        fails++;
    }
    char buf[64];
    const int n = time_sync_encode_response(7u, 42u, 100, 101, buf, sizeof(buf));
    if (n <= 0 || strcmp(buf, "TSR,7,42,100,101\n") != 0) {
        fails++;
    }
    if (time_sync_encode_response(7u, 42u, 101, 100, buf, sizeof(buf)) != -1) {
        fails++;
    }
    if (fails != 0) {
        ESP_LOGE(TAG, "self-test failed (%d)", fails);
        return -1;
    }
    ESP_LOGI(TAG, "self-test passed");
    return 0;
}
