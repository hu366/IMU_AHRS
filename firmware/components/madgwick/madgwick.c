/*
 * Madgwick IMU (6DoF) update.
 * Algorithm: S. O. H. Madgwick, 2011. Original C: MadgwickAHRS.c (x-io Technologies).
 * This file wraps the official IMU update with an instance and a real dt
 * (gyro in rad/s, accel as gravity direction in g).
 */

#include "madgwick.h"

#include <math.h>

static float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void madgwick_init(madgwick_t *filter, float beta)
{
    if (filter == NULL) {
        return;
    }
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->beta = beta;
}

void madgwick_update_imu(madgwick_t *filter,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    if (filter == NULL || dt <= 0.0f) {
        return;
    }

    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;
    const float beta = filter->beta;

    /* Rate of change of quaternion from gyroscope */
    q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    q_dot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    q_dot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    /* Feedback only if accelerometer measurement is valid */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
             + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
             + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        q_dot1 -= beta * s0;
        q_dot2 -= beta * s1;
        q_dot3 -= beta * s2;
        q_dot4 -= beta * s3;
    }

    q0 += q_dot1 * dt;
    q1 += q_dot2 * dt;
    q2 += q_dot3 * dt;
    q3 += q_dot4 * dt;

    recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q0 = q0 * recip_norm;
    filter->q1 = q1 * recip_norm;
    filter->q2 = q2 * recip_norm;
    filter->q3 = q3 * recip_norm;
}
