#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q0;   /* w */
    float q1;   /* x */
    float q2;   /* y */
    float q3;   /* z */
    float beta;
} madgwick_t;

void madgwick_init(madgwick_t *filter, float beta);
void madgwick_update_imu(madgwick_t *filter,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);

#ifdef __cplusplus
}
#endif
