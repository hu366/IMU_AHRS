from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from analysis.capture_ahrs import AhrsCsvCollector, extract_ahrs_log, parse_args
from analysis.madgwick_offline import MadgwickIMU, run_madgwick
from analysis.vqf_compare import (
    compare_series,
    load_ahrs_text,
    median_dt_s,
    quat_angle_deg,
    run_vqf_official,
    synthesize_ahrs,
    tilt_err_deg,
)

vqf = pytest.importorskip("vqf")

PC_ROOT = Path(__file__).resolve().parents[1]
VIEWER_REQ = PC_ROOT / "requirements.txt"
PYPROJECT = PC_ROOT / "pyproject.toml"
ANALYSIS_REQ = PC_ROOT / "analysis" / "requirements.txt"

MIXED_LOG = """
I (123) imu: gyro bias armed; starting AHRS realtime loop
I (124) imu: UART AHRS CSV dump, 120.0 s (4.2, bias_subtracted=1, algo=vqf-c)
# ahrs_csv source=esp32-c3 algo=vqf-c accel_unit=g gyro_unit=rad/s frame=hand bias_subtracted=1 nominal_fs=100
t_us,gx,gy,gz,ax,ay,az,qw,qx,qy,qz
I (200) wifi: something
10000,0.001,-0.002,0.0005,0.01,-0.02,1.00,1.0,0.0,0.0,0.0
20000,0.0012,-0.0018,0.0004,0.01,-0.02,0.99,0.9999,0.001,0.0,0.0
30000,0.0008,-0.0021,0.0006,0.01,-0.02,1.01,0.9998,0.002,0.0,0.0
I (61234) imu: ahrs CSV dump done n=3 target=12000
I (61240) imu: stats n=100 |q|=1.0000
"""


def test_vqf_not_in_viewer_runtime():
    text = VIEWER_REQ.read_text(encoding="utf-8").lower()
    assert "vqf" not in text
    deps_block = PYPROJECT.read_text(encoding="utf-8").split("[project.optional-dependencies]", 1)[0]
    assert "vqf" not in deps_block
    extra = ANALYSIS_REQ.read_text(encoding="utf-8")
    assert "vqf" in extra
    for src in (PC_ROOT / "imu_viewer").glob("*.py"):
        body = src.read_text(encoding="utf-8")
        assert "import vqf" not in body
        assert "from vqf" not in body
        assert "madgwick" not in body.lower()
    renderer = (PC_ROOT / "imu_viewer" / "renderer.py").read_text(encoding="utf-8")
    assert "quat_to_axis_up" in renderer
    assert "slerp" not in renderer.lower()


def test_extract_mixed_idf_log():
    col = extract_ahrs_log(MIXED_LOG)
    assert col.done is True
    assert col.n == 3
    assert col.rows[0].startswith("10000,")
    text = col.to_text()
    assert text.splitlines()[0].startswith("# ahrs_csv")
    assert "qw,qx,qy,qz" in text
    assert "wifi" not in text
    assert "stats n=" not in text


def test_collector_ignores_after_dump_done():
    col = AhrsCsvCollector()
    col.feed("t_us,gx,gy,gz,ax,ay,az,qw,qx,qy,qz")
    col.feed("10000,0.1,0.0,0.0,0,0,1,1,0,0,0")
    col.feed("I (1) imu: ahrs CSV dump done n=1 target=1")
    col.feed("20000,0.2,0.0,0.0,0,0,1,1,0,0,0")
    assert col.done is True
    assert col.n == 1


def test_from_log_cli():
    work = PC_ROOT / "analysis" / "out" / "_pytest_ahrs"
    work.mkdir(parents=True, exist_ok=True)
    src = work / "monitor.log"
    dst = work / "ahrs.csv"
    src.write_text(MIXED_LOG, encoding="utf-8")
    from analysis.capture_ahrs import main

    rc = main(["--from-log", str(src), "--out", str(dst), "--min-rows", "3"])
    assert rc == 0
    series = load_ahrs_text(dst.read_text(encoding="utf-8"))
    assert series.n == 3
    assert median_dt_s(series.t_us) == pytest.approx(0.01)
    assert series.meta["algo"] == "vqf-c"
    assert series.meta["bias_subtracted"] == "1"
    assert series.meta["accel_unit"] == "g"


def test_cli_requires_port_or_from_log():
    args = parse_args(["--from-log", "raw.txt", "--out", "analysis/data/ahrs.csv"])
    assert args.from_log is not None
    assert args.port is None


def test_madgwick_still_near_identity():
    filt = MadgwickIMU(0.1)
    for _ in range(200):
        filt.update(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.01)
    q = filt.quaternion()
    assert q[0] == pytest.approx(1.0, abs=1e-5)
    assert q[1:] == pytest.approx((0.0, 0.0, 0.0), abs=1e-5)


def test_madgwick_x_rotation_matches_gyro():
    n = 157
    gyr = np.zeros((n, 3))
    gyr[:, 0] = 1.0
    acc = np.zeros((n, 3))
    theta = 0.0
    dt = 0.01
    for i in range(n):
        theta += 1.0 * dt
        acc[i, 1] = np.sin(theta)
        acc[i, 2] = np.cos(theta)
    q = run_madgwick(gyr, acc, dt, beta=0.1)
    # ~90 deg about X: w and x dominate
    assert abs(q[-1, 0]) == pytest.approx(abs(q[-1, 1]), rel=0.15)
    assert abs(q[-1, 2]) < 0.15
    assert abs(q[-1, 3]) < 0.15


def test_synthetic_still_fs_from_t_us():
    series = synthesize_ahrs("still", fs_hz=100.0, seed=0)
    assert median_dt_s(series.t_us) == pytest.approx(0.01)
    assert 1.0 / median_dt_s(series.t_us) == pytest.approx(100.0)


def test_compare_still_and_write_table():
    series = synthesize_ahrs("still", fs_hz=100.0, seed=1)
    metrics = compare_series(series)
    assert metrics["fs_hz"] == pytest.approx(100.0)
    assert metrics["still_tilt_std_deg_vqf"] <= metrics["still_tilt_std_deg_madgwick"] * 1.5
    out = PC_ROOT / "analysis" / "out"
    out.mkdir(parents=True, exist_ok=True)
    from analysis.vqf_compare import main

    rc = main(["--synthetic", "still", "--out", str(out)])
    assert rc == 0
    table = (out / "vqf_vs_madgwick.csv").read_text(encoding="utf-8")
    assert "still_tilt_std_deg" in table
    assert "yaw_drift_deg_per_min" in table
    assert "static_tilt_err_deg" in table
    note = (out / "vqf_compare.txt").read_text(encoding="utf-8")
    assert "yaw around gravity" in note.lower() or "yaw" in note.lower()


def test_vqf_yaw_drift_better_than_madgwick_on_residual_bias():
    series = synthesize_ahrs("bias_still", fs_hz=100.0, seed=2)
    metrics = compare_series(series)
    madg = abs(float(metrics["yaw_drift_deg_per_min_madgwick"]))
    vqf_d = abs(float(metrics["yaw_drift_deg_per_min_vqf"]))
    assert madg > 5.0
    assert vqf_d < madg * 0.5


def test_xrot_gravity_alignment():
    series = synthesize_ahrs("xrot", fs_hz=100.0, seed=0)
    metrics = compare_series(series)
    q_v = metrics["q_vqf"]
    err = tilt_err_deg(q_v, series.acc_g)
    # after the rotation, quasi-static tail should re-align
    assert float(np.mean(err[-50:])) < 5.0


def test_port_error_uses_device_quat():
    series = synthesize_ahrs("still", fs_hz=100.0, seed=0)
    metrics = compare_series(series)
    series.q_esp = metrics["q_vqf"].copy()
    metrics2 = compare_series(series)
    assert metrics2["port_err_mean_deg"] < 0.05
    assert metrics2["port_used_conjugate"] == 0.0

    series = synthesize_ahrs("xrot", fs_hz=100.0, seed=0)
    metrics = compare_series(series)
    series.q_esp = metrics["q_vqf"].copy()
    series.q_esp[:, 1:] *= -1.0
    metrics3 = compare_series(series)
    assert metrics3["port_err_mean_deg"] < 0.5
    assert metrics3["port_used_conjugate"] == 1.0


def test_port_uses_firmware_nominal_ts_when_dump_is_slow():
    """UART dump can stretch the loop; q_esp was produced with Ts=1/nominal_fs."""
    series = synthesize_ahrs("xrot", fs_hz=100.0, seed=0)
    series.t_us = np.rint(series.t_us.astype(np.float64) * 1.3).astype(np.int64)
    series.q_esp = run_vqf_official(series.gyr, series.acc_g, 0.01)
    series.meta["nominal_fs"] = "100"
    metrics = compare_series(series)
    assert median_dt_s(series.t_us) == pytest.approx(0.013, rel=0.05)
    assert metrics["port_ts_s"] == pytest.approx(0.01)
    assert metrics["port_err_mean_deg"] < 0.05
    assert metrics["port_used_conjugate"] == 0.0


def test_quat_angle_identical_is_zero():
    q = np.array([[1.0, 0.0, 0.0, 0.0], [0.7071, 0.7071, 0.0, 0.0]])
    assert quat_angle_deg(q, q) == pytest.approx([0.0, 0.0], abs=1e-6)
    qn = q.copy()
    qn *= -1.0
    assert quat_angle_deg(q, qn) == pytest.approx([0.0, 0.0], abs=1e-6)
