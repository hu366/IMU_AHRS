"""Hamilton quaternion helpers and VPython axis/up conversion (SH-02).

Hand frame h (already applied on the firmware):
  X_h  wrist -> fingers
  Z_h  out of the back of the hand
  Y_h  right-hand rule

VPython box.axis is body X, box.up is body Y. No extra axis swap here.
"""

from __future__ import annotations

import math

Quaternion = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]

IDENTITY: Quaternion = (1.0, 0.0, 0.0, 0.0)

_ZERO_NORM = 1e-12


def normalize(q: Quaternion) -> Quaternion:
    w, x, y, z = q
    mag = math.sqrt(w * w + x * x + y * y + z * z)
    if mag < _ZERO_NORM:
        raise ValueError("cannot normalize a near-zero quaternion")
    return (w / mag, x / mag, y / mag, z / mag)


def conjugate(q: Quaternion) -> Quaternion:
    w, x, y, z = q
    return (w, -x, -y, -z)


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def rotate_vector(q: Quaternion, v: Vec3) -> Vec3:
    """Rotate vector v by unit quaternion q (active / body-to-world)."""
    qn = normalize(q)
    qv: Quaternion = (0.0, v[0], v[1], v[2])
    _, x, y, z = multiply(multiply(qn, qv), conjugate(qn))
    return (x, y, z)


def quat_to_axis_up(q: Quaternion) -> tuple[Vec3, Vec3]:
    """Map Hamilton (w,x,y,z) to VPython box axis (body X) and up (body Y)."""
    axis = rotate_vector(q, (1.0, 0.0, 0.0))
    up = rotate_vector(q, (0.0, 1.0, 0.0))
    return axis, up
