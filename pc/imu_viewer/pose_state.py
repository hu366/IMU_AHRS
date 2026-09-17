"""Thread-safe latest-pose slot. BLE/asyncio writes; render thread copies."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Sequence

from imu_viewer.orientation import IDENTITY
from imu_viewer.protocol import ProtocolDecoder, Quaternion

Clock = Callable[[], float]


@dataclass(frozen=True)
class PoseState:
    quaternion: Quaternion
    received_at_monotonic: float
    sequence: int | None
    connected: bool


@dataclass(frozen=True)
class PoseSnapshot:
    pose: PoseState
    has_pose: bool
    stale: bool
    age_s: float | None
    frames_ok: int
    frames_drop: int
    parse_errors: int
    receive_hz: float
    mode: str


class PoseStore:
    """Overwrite-only latest pose. Never queues frames for the renderer."""

    def __init__(
        self,
        timeout_s: float = 1.0,
        clock: Clock = time.monotonic,
        mode: str = "ble",
    ) -> None:
        self._timeout_s = timeout_s
        self._clock = clock
        self._mode = mode
        self._lock = threading.Lock()
        self._latest: PoseState | None = None
        self._has_pose = False
        self._connected = False
        self._frames_ok = 0
        self._frames_drop = 0
        self._parse_errors = 0
        self._recv_times: deque[float] = deque()

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self._mode = mode

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = connected
            if self._latest is not None:
                self._latest = PoseState(
                    quaternion=self._latest.quaternion,
                    received_at_monotonic=self._latest.received_at_monotonic,
                    sequence=self._latest.sequence,
                    connected=connected,
                )

    def ingest(
        self,
        frames: Sequence[Quaternion],
        *,
        decoder: ProtocolDecoder | None = None,
        sequence: int | None = None,
    ) -> None:
        """Store only the last quaternion from this batch (no history queue)."""
        now = self._clock()
        with self._lock:
            if decoder is not None:
                self._frames_ok = decoder.frames_ok
                self._frames_drop = decoder.frames_drop
                self._parse_errors = decoder.parse_errors
            elif frames:
                self._frames_ok += len(frames)
            if not frames:
                self._trim_recv_times(now)
                return
            q = frames[-1]
            self._latest = PoseState(
                quaternion=q,
                received_at_monotonic=now,
                sequence=sequence,
                connected=self._connected,
            )
            self._has_pose = True
            self._recv_times.append(now)
            self._trim_recv_times(now)

    def snapshot(self) -> PoseSnapshot:
        """Copy state under the lock, then release. No drawing in here."""
        now = self._clock()
        with self._lock:
            latest = self._latest
            has_pose = self._has_pose
            connected = self._connected
            frames_ok = self._frames_ok
            frames_drop = self._frames_drop
            parse_errors = self._parse_errors
            mode = self._mode
            self._trim_recv_times(now)
            receive_hz = float(len(self._recv_times))
        if latest is None:
            pose = PoseState(
                quaternion=IDENTITY,
                received_at_monotonic=now,
                sequence=None,
                connected=connected,
            )
            return PoseSnapshot(
                pose=pose,
                has_pose=False,
                stale=True,
                age_s=None,
                frames_ok=frames_ok,
                frames_drop=frames_drop,
                parse_errors=parse_errors,
                receive_hz=receive_hz,
                mode=mode,
            )
        age_s = now - latest.received_at_monotonic
        stale = age_s > self._timeout_s
        pose = PoseState(
            quaternion=latest.quaternion,
            received_at_monotonic=latest.received_at_monotonic,
            sequence=latest.sequence,
            connected=connected,
        )
        return PoseSnapshot(
            pose=pose,
            has_pose=has_pose,
            stale=stale,
            age_s=age_s,
            frames_ok=frames_ok,
            frames_drop=frames_drop,
            parse_errors=parse_errors,
            receive_hz=receive_hz,
            mode=mode,
        )

    def _trim_recv_times(self, now: float) -> None:
        cutoff = now - 1.0
        times = self._recv_times
        while times and times[0] < cutoff:
            times.popleft()


def format_status(snap: PoseSnapshot) -> str:
    if snap.mode == "demo":
        conn = "demo"
    elif snap.pose.connected:
        conn = "connected"
    else:
        conn = "disconnected"
    if not snap.has_pose:
        data = "no pose yet"
    elif snap.stale:
        age = snap.age_s if snap.age_s is not None else 0.0
        data = f"STALE {age:.1f}s, last pose held"
    else:
        age_ms = (snap.age_s or 0.0) * 1000.0
        data = f"age {age_ms:.0f} ms"
    return (
        f"{conn} | {data} | {snap.receive_hz:.0f} Hz | "
        f"ok={snap.frames_ok} drop={snap.frames_drop} err={snap.parse_errors}"
    )
