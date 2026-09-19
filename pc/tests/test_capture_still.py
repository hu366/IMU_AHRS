from __future__ import annotations

from pathlib import Path

from analysis.capture_still import StillCsvCollector, extract_still_log, parse_args

PC_ROOT = Path(__file__).resolve().parents[1]
VIEWER_REQ = PC_ROOT / "requirements.txt"
PYPROJECT = PC_ROOT / "pyproject.toml"


MIXED_LOG = """
I (123) imu: keep still for UART CSV dump, 60.0 s (3.3 Allan, bias_subtracted=0)
# still_csv source=esp32-c3 accel_unit=g gyro_unit=rad/s frame=hand bias_subtracted=0
t_us,gx,gy,gz,ax,ay,az
I (200) wifi: something
10000,0.001,-0.002,0.0005,0.01,-0.02,1.00
20000,0.0012,-0.0018,0.0004,0.01,-0.02,0.99
30000,0.0008,-0.0021,0.0006,0.01,-0.02,1.01
I (61234) imu: still CSV dump done n=3 target=6000
I (61240) imu: keep still for gyro bias, 3.0 s
"""


def test_pyserial_not_in_viewer_runtime():
    text = VIEWER_REQ.read_text(encoding="utf-8").lower()
    assert "pyserial" not in text
    deps_block = PYPROJECT.read_text(encoding="utf-8").split("[project.optional-dependencies]", 1)[0]
    assert "pyserial" not in deps_block


def test_extract_mixed_idf_log():
    col = extract_still_log(MIXED_LOG)
    assert col.done is True
    assert col.n == 3
    assert col.rows[0].startswith("10000,")
    text = col.to_text()
    assert text.splitlines()[0].startswith("# still_csv")
    assert "t_us,gx,gy,gz,ax,ay,az" in text
    assert "keep still for gyro bias" not in text
    assert "wifi" not in text


def test_collector_ignores_after_dump_done():
    col = StillCsvCollector()
    col.feed("t_us,gx,gy,gz,ax,ay,az")
    col.feed("10000,0.1,0.0,0.0,0,0,1")
    col.feed("I (1) imu: still CSV dump done n=1 target=1")
    col.feed("20000,0.2,0.0,0.0,0,0,1")
    assert col.done is True
    assert col.n == 1


def test_from_log_cli():
    # Avoid pytest tmp_path (Windows permission on this machine); use analysis/out.
    work = PC_ROOT / "analysis" / "out" / "_pytest_capture"
    work.mkdir(parents=True, exist_ok=True)
    src = work / "monitor.log"
    dst = work / "still.log"
    src.write_text(MIXED_LOG, encoding="utf-8")
    from analysis.capture_still import main

    rc = main(["--from-log", str(src), "--out", str(dst), "--min-rows", "3"])
    assert rc == 0
    out = dst.read_text(encoding="utf-8")
    assert out.count("\n") >= 4
    assert "0.001,-0.002" in out


def test_cli_requires_port_or_from_log():
    args = parse_args(["--from-log", "raw.txt", "--out", "analysis/data/still.log"])
    assert args.from_log is not None
    assert args.port is None


def test_console_flag_exists():
    args = parse_args(["--console", "--port", "COM5"])
    assert args.console is True
    assert args.port == "COM5"
