#pragma once

#include <stddef.h>

#include "imu_types.h"

#define POSE_PROTOCOL_BUF_LEN  128

/* 成功返回写入字节数（含 '\n'，不含 '\0'）；失败返回 -1，buf 不保证可用 */
int pose_protocol_encode(const quat_t *q, char *buf, size_t buf_len);

/* Encode a quaternion together with the source IMU sample timestamp. */
int pose_protocol_encode_timed(const quat_t *q,
                               uint32_t boot_id,
                               uint32_t sequence,
                               int64_t t_data_ready_us,
                               char *buf,
                               size_t buf_len);

/* Boot-time check of SH-01 vectors. Returns 0 on match. */
int pose_protocol_self_test(void);
