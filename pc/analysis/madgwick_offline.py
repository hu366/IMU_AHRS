"""Firmware-matching Madgwick 6DoF (IMU only). PC offline use.

Formulas, beta, and dt handling follow firmware/components/madgwick/madgwick.c.
Gyro: rad/s. Accel: gravity direction in g (normalized inside the update).
"""

from __future__ import annotations

import math

import numpy as np

DEFAULT_BETA = 0.1


def _inv_sqrt(x: float) -> float:
    return 1.0 / math.sqrt(x)


class MadgwickIMU:
    def __init__(self, beta: float = DEFAULT_BETA) -> None:
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.beta = float(beta)

    def quaternion(self) -> tuple[float, float, float, float]:
        return (self.q0, self.q1, self.q2, self.q3)

    def update(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        dt: float,
    ) -> tuple[float, float, float, float]:
        if dt <= 0.0:
            return self.quaternion()

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        beta = self.beta

        q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            recip_norm = _inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            _2q0 = 2.0 * q0
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _4q0 = 4.0 * q0
            _4q1 = 4.0 * q1
            _4q2 = 4.0 * q2
            _8q1 = 8.0 * q1
            _8q2 = 8.0 * q2
            q0q0 = q0 * q0
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = (
                _4q1 * q3q3
                - _2q3 * ax
                + 4.0 * q0q0 * q1
                - _2q0 * ay
                - _4q1
                + _8q1 * q1q1
                + _8q1 * q2q2
                + _4q1 * az
            )
            s2 = (
                4.0 * q0q0 * q2
                + _2q0 * ax
                + _4q2 * q3q3
                - _2q3 * ay
                - _4q2
                + _8q2 * q1q1
                + _8q2 * q2q2
                + _4q2 * az
            )
            s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
            s_n2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3
            if s_n2 > 0.0:
                recip_norm = _inv_sqrt(s_n2)
                s0 *= recip_norm
                s1 *= recip_norm
                s2 *= recip_norm
                s3 *= recip_norm
                q_dot1 -= beta * s0
                q_dot2 -= beta * s1
                q_dot3 -= beta * s2
                q_dot4 -= beta * s3

        q0 += q_dot1 * dt
        q1 += q_dot2 * dt
        q2 += q_dot3 * dt
        q3 += q_dot4 * dt

        recip_norm = _inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q0 = q0 * recip_norm
        self.q1 = q1 * recip_norm
        self.q2 = q2 * recip_norm
        self.q3 = q3 * recip_norm
        return self.quaternion()


def run_madgwick(
    gyr: np.ndarray,
    acc_g: np.ndarray,
    dt: np.ndarray | float,
    beta: float = DEFAULT_BETA,
) -> np.ndarray:
    """Step Madgwick on (N,3) gyro rad/s and accel g. dt is scalar or (N,)."""
    n = int(gyr.shape[0])
    if np.isscalar(dt):
        dts = np.full(n, float(dt), dtype=float)
    else:
        dts = np.asarray(dt, dtype=float)
        if dts.shape == (n - 1,):
            dts = np.concatenate([[dts[0]], dts])
    filt = MadgwickIMU(beta)
    out = np.zeros((n, 4), dtype=float)
    for i in range(n):
        out[i] = filt.update(
            float(gyr[i, 0]),
            float(gyr[i, 1]),
            float(gyr[i, 2]),
            float(acc_g[i, 0]),
            float(acc_g[i, 1]),
            float(acc_g[i, 2]),
            float(dts[i]),
        )
    return out
