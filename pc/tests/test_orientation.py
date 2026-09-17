from __future__ import annotations

import math

import pytest

from imu_viewer.orientation import (
    IDENTITY,
    conjugate,
    multiply,
    normalize,
    quat_to_axis_up,
    rotate_vector,
)

SQRT_HALF = math.sqrt(2.0) / 2.0


def _vec_close(actual, expected, abs_=1e-9):
    assert actual == pytest.approx(expected, abs=abs_)


def test_identity_axis_up_matches_initial_pose():
    axis, up = quat_to_axis_up(IDENTITY)
    _vec_close(axis, (1.0, 0.0, 0.0))
    _vec_close(up, (0.0, 1.0, 0.0))


def test_rotate_x_90_deg():
    q = (SQRT_HALF, SQRT_HALF, 0.0, 0.0)
    axis, up = quat_to_axis_up(q)
    _vec_close(axis, (1.0, 0.0, 0.0))
    _vec_close(up, (0.0, 0.0, 1.0))
    z = rotate_vector(q, (0.0, 0.0, 1.0))
    _vec_close(z, (0.0, -1.0, 0.0))


def test_rotate_y_90_deg():
    q = (SQRT_HALF, 0.0, SQRT_HALF, 0.0)
    axis, up = quat_to_axis_up(q)
    _vec_close(axis, (0.0, 0.0, -1.0))
    _vec_close(up, (0.0, 1.0, 0.0))


def test_rotate_z_90_deg():
    q = (SQRT_HALF, 0.0, 0.0, SQRT_HALF)
    axis, up = quat_to_axis_up(q)
    _vec_close(axis, (0.0, 1.0, 0.0))
    _vec_close(up, (-1.0, 0.0, 0.0))


def test_normalize_scales_to_unit():
    q = normalize((2.0, 0.0, 0.0, 0.0))
    assert q == pytest.approx(IDENTITY)


def test_normalize_rejects_zero():
    with pytest.raises(ValueError):
        normalize((0.0, 0.0, 0.0, 0.0))


def test_conjugate_and_multiply_identity():
    q = (0.9981, 0.0123, -0.0310, 0.0512)
    qn = normalize(q)
    back = multiply(qn, conjugate(qn))
    assert back == pytest.approx(IDENTITY, abs=1e-9)
