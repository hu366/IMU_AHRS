from __future__ import annotations

import ast
import inspect
import logging
from pathlib import Path

import pytest

from imu_viewer.ble_client import BleClient, NotifyHandler
from imu_viewer.clock_sync import AffineClockMapper
from imu_viewer.pose_state import PoseStore

_PC_ROOT = Path(__file__).resolve().parents[1]


def test_fake_notify_fragments_update_store():
    store = PoseStore()
    store.set_connected(True)
    handler = NotifyHandler(store)
    handler.on_notify(None, b"Q,1.0")
    assert store.snapshot().has_pose is False
    handler.on_notify(None, b"000,0.0000,0.0000,0.0000\n")
    snap = store.snapshot()
    assert snap.has_pose is True
    assert snap.pose.quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert handler.decoder.frames_ok == 1


def test_fake_notify_sticky_and_bad_then_good():
    store = PoseStore()
    printed: list[tuple[float, float, float, float]] = []
    handler = NotifyHandler(store, on_frame=lambda q: printed.append(q))
    handler.on_notify(
        "tx",
        b"Q,abc,0,0,0\nQ,1.0000,0.0000,0.0000,0.0000\nQ,0.9981,0.0123,-0.0310,0.0512\n",
    )
    snap = store.snapshot()
    assert snap.has_pose is True
    assert len(printed) == 2
    assert handler.decoder.frames_drop == 1
    assert handler.decoder.frames_ok == 2
    w, x, y, z = snap.pose.quaternion
    assert abs(w - 0.9981) < 1e-3


def test_ble_client_handle_notify_is_protocol_feed():
    store = PoseStore()
    client = BleClient(store, print_frames=False)
    client.handle_notify("char", b"Q,1.0000,0.0000,0.0000,0.0000\n")
    assert store.snapshot().has_pose is True
    assert client.decoder.frames_ok == 1


def test_notify_captures_t4_and_dispatches_tsr_to_clock_mapper():
    values = iter([2_001_000])
    mapper = AffineClockMapper(min_samples=2)
    client = BleClient(
        PoseStore(),
        print_frames=False,
        clock_mapper=mapper,
        monotonic_ns=lambda: next(values),
    )
    client._pending_sync[12] = 1_999_000
    client.handle_notify("char", b"TSR,9,12,1000,1001\n")
    assert mapper.quality().boot_id == 9
    assert mapper.exchanges[0].t4_pc_ns == 2_001_000
    assert mapper.exchanges[0].t1_pc_ns == 1_999_000


def test_sync_report_prints_quality_fields(caplog):
    values = iter([2_001_000])
    mapper = AffineClockMapper(min_samples=2)
    client = BleClient(
        PoseStore(),
        print_frames=False,
        clock_mapper=mapper,
        sync_report=True,
        monotonic_ns=lambda: next(values),
    )
    client._pending_sync[12] = 1_999_000
    with caplog.at_level(logging.INFO, logger="imu_viewer.ble"):
        client.handle_notify("char", b"TSR,9,12,1000,1001\n")
    assert "SYNC boot_id=9 id=12" in caplog.text
    assert "ready=False" in caplog.text
    assert "write_call_ms=" in caplog.text
    assert "esp_service_ms=" in caplog.text
    assert "rtt_min_ms=" in caplog.text
    assert "residual_rms_ms=" in caplog.text


def test_qt_new_boot_invalidates_old_clock_mapping_and_keeps_pose_display():
    mapper = AffineClockMapper(min_samples=2)
    store = PoseStore()
    client = BleClient(store, print_frames=False, clock_mapper=mapper)
    client.handle_notify("char", b"QT,77,3,456,1,0,0,0\n")
    assert mapper.boot_id == 77
    snap = store.snapshot()
    assert snap.has_pose is True
    assert snap.pose.sequence == 3


def test_ble_and_protocol_modules_do_not_import_vpython():
    for module in ("ble_client.py", "protocol.py", "pose_state.py", "orientation.py"):
        src = (_PC_ROOT / "imu_viewer" / module).read_text(encoding="utf-8")
        tree = ast.parse(src)
        for node in ast.walk(tree):
            names: list[str] = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            for name in names:
                root = name.split(".")[0]
                assert root != "vpython", f"{module} imports vpython"
                assert root != "renderer", f"{module} imports renderer"


def test_notify_callback_source_has_no_vpython():
    src = inspect.getsource(NotifyHandler.on_notify) + inspect.getsource(BleClient.handle_notify)
    lowered = src.lower()
    assert "vpython" not in lowered
    assert "canvas" not in lowered
    assert "box.axis" not in lowered
