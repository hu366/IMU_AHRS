from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from analysis.allan_analysis import (
    BLOCKED_BANNER,
    is_synthetic,
    load_still_text,
    median_dt_s,
    parse_args,
    parse_comment_meta,
    still_to_csv_text,
    strip_ansi,
    synthesize_still,
)

PC_ROOT = Path(__file__).resolve().parents[1]
VIEWER_REQ = PC_ROOT / "requirements.txt"
PYPROJECT = PC_ROOT / "pyproject.toml"
ANALYSIS_REQ = PC_ROOT / "analysis" / "requirements.txt"
SYNTH_DATA = PC_ROOT / "analysis" / "data" / "synthetic_still.csv"


def test_viewer_requirements_do_not_include_allan():
    text = VIEWER_REQ.read_text(encoding="utf-8").lower()
    assert "allan-variance" not in text
    assert "allan_variance" not in text
    proj = PYPROJECT.read_text(encoding="utf-8")
    deps_block = proj.split("[project.optional-dependencies]", 1)[0]
    assert "allan-variance" not in deps_block
    assert "pyserial" not in deps_block
    extra = ANALYSIS_REQ.read_text(encoding="utf-8")
    assert "allan-variance" in extra
    assert "pyserial" in extra


def test_strip_ansi_and_comment_meta():
    raw = "\x1b[0;32m# source=esp32-c3 accel_unit=g gyro_unit=rad/s\x1b[0m"
    meta = parse_comment_meta(strip_ansi(raw) + "\n# bias_subtracted=0\n")
    assert meta["source"] == "esp32-c3"
    assert meta["accel_unit"] == "g"
    assert meta["bias_subtracted"] == "0"


def test_load_mixed_uart_log_uses_t_us_not_pc_clock():
    log = """
I (123) imu: keep still for UART CSV dump
# still_csv source=esp32-c3 accel_unit=g gyro_unit=rad/s frame=hand bias_subtracted=0
t_us,gx,gy,gz,ax,ay,az
I (200) wifi: something
10000,0.001,-0.002,0.0005,0.01,-0.02,1.00
\x1b[0;32m20000,0.0012,-0.0018,0.0004,0.01,-0.02,0.99\x1b[0m
I (210) imu: not a csv line
30000,0.0008,-0.0021,0.0006,0.01,-0.02,1.01
"""
    series = load_still_text(log, source_path=Path("still.log"))
    assert series.n == 3
    assert list(series.t_us) == [10000, 20000, 30000]
    dt = median_dt_s(series.t_us)
    assert dt == pytest.approx(0.01)
    assert 1.0 / dt == pytest.approx(100.0)
    assert series.meta["source"] == "esp32-c3"
    assert not is_synthetic(series.meta, Path("still.log"), False)


def test_synthetic_marker_from_name_and_header():
    assert is_synthetic({"source": "synthetic"}, None, False)
    assert is_synthetic({}, Path("synthetic_still.csv"), False)
    assert not is_synthetic({"source": "esp32-c3"}, Path("still.csv"), False)
    assert is_synthetic({"source": "esp32-c3"}, Path("still.csv"), True)


def test_synthesize_dt_from_t_us():
    series = synthesize_still(n=200, fs_hz=100.0, sigma=0.002, seed=0)
    assert series.n == 200
    assert median_dt_s(series.t_us) == pytest.approx(0.01)
    assert series.meta["source"] == "synthetic"
    text = still_to_csv_text(series)
    assert "source=synthetic" in text
    assert "do not copy Allan" in text
    reloaded = load_still_text(text)
    assert reloaded.n == 200
    assert median_dt_s(reloaded.t_us) == pytest.approx(0.01)


def test_cli_requires_input_or_synthesize():
    with pytest.raises(SystemExit):
        parse_args(["--help"])
    args = parse_args(["--synthesize", "--out", "analysis/out"])
    assert args.synthesize is True


@pytest.mark.skipif(not SYNTH_DATA.is_file(), reason="committed synthetic CSV missing")
def test_committed_synthetic_csv_dt():
    from analysis.allan_analysis import load_still_csv

    series = load_still_csv(SYNTH_DATA)
    assert series.n >= 1000
    assert median_dt_s(series.t_us) == pytest.approx(0.01, rel=1e-3)
    assert is_synthetic(series.meta, SYNTH_DATA, False)


def test_full_pipeline_synthetic_blocked():
    pytest.importorskip("allan_variance")
    pytest.importorskip("numpy")
    from analysis.allan_analysis import main

    out = PC_ROOT / "analysis" / "out" / "_pytest_synth"
    if out.exists():
        shutil.rmtree(out)
    rc = main(["--synthesize", "--out", str(out), "--bias-s", "3.0"])
    assert rc == 0
    adev = (out / "allan_adev.csv").read_text(encoding="utf-8")
    params = (out / "allan_params.csv").read_text(encoding="utf-8")
    zupt = (out / "zupt_suggestion.txt").read_text(encoding="utf-8")
    synth = (out / "synthetic_still.csv").read_text(encoding="utf-8")
    assert adev.startswith("tau_s,adev_gx,adev_gy,adev_gz")
    assert "dt_s:" in zupt
    assert "median diff of t_us" in zupt
    assert "PC wall clock" in zupt
    assert "white_emp_gx:" in zupt
    assert "bias_instability_gx_rad_s:" in zupt
    assert "APP_GYRO_BIAS_S_current: 3" in zupt
    assert "APP_ZUPT_GYRO_RAD" in zupt
    assert BLOCKED_BANNER.strip() in zupt
    assert "APP_ZUPT_GYRO_RAD: (blocked)" in zupt
    assert "flicker" in params
    assert "source=synthetic" in synth
    assert "dt_s: 0.01" in zupt.replace("0.010000", "0.01")
    shutil.rmtree(out, ignore_errors=True)


def test_device_csv_is_not_blocked():
    pytest.importorskip("allan_variance")
    pytest.importorskip("numpy")
    from analysis.allan_analysis import main, synthesize_still, still_to_csv_text

    work = PC_ROOT / "analysis" / "out" / "_pytest_device"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    series = synthesize_still(n=800, seed=1)
    series.meta["source"] = "esp32-c3"
    series.meta["accel_unit"] = "g"
    csv_path = work / "still.csv"
    csv_path.write_text(
        still_to_csv_text(series).replace("source=synthetic", "source=esp32-c3"),
        encoding="utf-8",
    )
    out = work / "out"
    rc = main(["--input", str(csv_path), "--out", str(out)])
    assert rc == 0
    zupt = (out / "zupt_suggestion.txt").read_text(encoding="utf-8")
    assert "BLOCKED" not in zupt
    assert "APP_ZUPT_GYRO_RAD: (blocked)" not in zupt
    assert "APP_ZUPT_GYRO_RAD:" in zupt
    assert "source: esp32-c3" in zupt
    shutil.rmtree(work, ignore_errors=True)
