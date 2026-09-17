from __future__ import annotations

import math
import threading
import time

import pytest

from imu_viewer.pose_state import PoseStore, format_status
from imu_viewer.protocol import ProtocolDecoder


class FakeClock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def test_keeps_only_latest_frame():
    store = PoseStore()
    store.set_connected(True)
    store.ingest([(1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0)])
    snap = store.snapshot()
    assert snap.pose.quaternion == pytest.approx((0.0, 1.0, 0.0, 0.0))
    assert snap.has_pose
    assert snap.pose.connected is True


def test_snapshot_is_a_copy():
    store = PoseStore()
    store.ingest([(1.0, 0.0, 0.0, 0.0)])
    a = store.snapshot()
    store.ingest([(0.0, 1.0, 0.0, 0.0)])
    b = store.snapshot()
    assert a.pose.quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert b.pose.quaternion == pytest.approx((0.0, 1.0, 0.0, 0.0))


def test_stale_uses_injected_monotonic_clock():
    clock = FakeClock(10.0)
    store = PoseStore(timeout_s=0.5, clock=clock)
    store.set_connected(True)
    store.ingest([(1.0, 0.0, 0.0, 0.0)])
    clock.t = 10.4
    snap = store.snapshot()
    assert snap.stale is False
    clock.t = 10.6
    snap = store.snapshot()
    assert snap.stale is True
    assert snap.has_pose is True
    assert snap.pose.quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert "STALE" in format_status(snap)
    assert "last pose held" in format_status(snap)


def test_never_received_uses_identity_and_no_pose():
    store = PoseStore()
    snap = store.snapshot()
    assert snap.has_pose is False
    assert snap.pose.quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert "no pose yet" in format_status(snap)


def test_disconnect_keeps_last_quaternion():
    store = PoseStore()
    store.set_connected(True)
    store.ingest([(0.0, 1.0, 0.0, 0.0)])
    store.set_connected(False)
    snap = store.snapshot()
    assert snap.pose.connected is False
    assert snap.pose.quaternion == pytest.approx((0.0, 1.0, 0.0, 0.0))
    assert snap.has_pose is True


def test_ingest_copies_decoder_stats():
    dec = ProtocolDecoder()
    dec.feed(b"Q,abc,0,0,0\nQ,1.0000,0.0000,0.0000,0.0000\n")
    store = PoseStore()
    store.ingest([(1.0, 0.0, 0.0, 0.0)], decoder=dec)
    snap = store.snapshot()
    assert snap.frames_ok == 1
    assert snap.frames_drop == 1
    assert snap.parse_errors == 1


def test_concurrent_writer_reader_sees_unit_quaternions():
    store = PoseStore()
    stop = threading.Event()

    def writer() -> None:
        i = 0
        while not stop.is_set():
            half = 0.01 * i
            q = (math.cos(half), math.sin(half), 0.0, 0.0)
            store.ingest((q,))
            i += 1

    thread = threading.Thread(target=writer)
    thread.start()
    try:
        for _ in range(200):
            snap = store.snapshot()
            q = snap.pose.quaternion
            mag = math.sqrt(sum(c * c for c in q))
            assert mag == pytest.approx(1.0, abs=1e-9)
            time.sleep(0.0005)
    finally:
        stop.set()
        thread.join(timeout=1.0)
