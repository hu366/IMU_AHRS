from __future__ import annotations

import math

import pytest

from imu_viewer.main import demo_quaternion, parse_args


def test_help_exits_zero():
    with pytest.raises(SystemExit) as ei:
        parse_args(["--help"])
    assert ei.value.code == 0


def test_demo_flag_and_name():
    args = parse_args(["--demo", "--name", "IMU-AHRS"])
    assert args.demo is True
    assert args.name == "IMU-AHRS"
    assert args.timeout == 1.0


def test_demo_quaternion_identity_then_x_90():
    q0 = demo_quaternion(0.0)
    assert q0[0] == pytest.approx(1.0)
    assert q0[1:] == pytest.approx((0.0, 0.0, 0.0))
    # 2 s into the 8 s X cycle is 90° about X: θ=π/2, q=(√2/2, √2/2, 0, 0)
    q90 = demo_quaternion(2.0)
    half = math.sqrt(2.0) / 2.0
    assert q90 == pytest.approx((half, half, 0.0, 0.0), abs=1e-9)
