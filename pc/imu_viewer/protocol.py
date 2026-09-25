"""Framed BLE text protocol for legacy and timestamped IMU messages."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TypeAlias

Quaternion: TypeAlias = tuple[float, float, float, float]
_MIN_NORM = 1e-6
_MAX_NORM = 2.0
_U32_MAX = (1 << 32) - 1
_I64_MIN = -(1 << 63)
_I64_MAX = (1 << 63) - 1


@dataclass(frozen=True)
class TimedQuaternionFrame:
    boot_id: int
    sequence: int
    t_esp_us: int
    quaternion_wxyz: Quaternion

    @property
    def q(self) -> Quaternion:
        return self.quaternion_wxyz

    @property
    def timestamp_esp_us(self) -> int:
        return self.t_esp_us

    @property
    def t_data_ready_us(self) -> int:
        return self.t_esp_us

    @property
    def quaternion(self) -> Quaternion:
        return self.quaternion_wxyz


TimedQuaternion = TimedQuaternionFrame


@dataclass(frozen=True)
class TimeSyncResponse:
    boot_id: int
    sync_id: int
    t2_receive_us: int
    t3_send_us: int

    @property
    def t2_esp_us(self) -> int:
        return self.t2_receive_us

    @property
    def t3_esp_us(self) -> int:
        return self.t3_send_us


SyncResponse = TimeSyncResponse
ProtocolEvent: TypeAlias = Quaternion | TimedQuaternionFrame | TimeSyncResponse


class ProtocolDecoder:
    """Append BLE fragments and parse Q, QT, and TSR lines."""

    def __init__(self, max_buffer: int = 4096) -> None:
        if max_buffer < 16:
            raise ValueError("max_buffer must be at least 16 bytes")
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.frames_ok = 0
        self.frames_drop = 0
        self.parse_errors = 0
        self.timed_frames_ok = 0
        self.sync_frames_ok = 0
        self.last_events: tuple[ProtocolEvent, ...] = ()

    def feed(self, data: bytes) -> list[Quaternion]:
        """Legacy API: return valid Q and QT quaternions."""
        events = self.feed_events(data)
        return [
            event.quaternion_wxyz if isinstance(event, TimedQuaternionFrame) else event
            for event in events
            if isinstance(event, (tuple, TimedQuaternionFrame))
        ]

    def feed_events(self, data: bytes | bytearray | memoryview) -> list[ProtocolEvent]:
        """Return all valid events in notification order."""
        if not data:
            self.last_events = ()
            return []
        self._buf.extend(bytes(data))
        out: list[ProtocolEvent] = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                if len(self._buf) > self._max_buffer:
                    self._buf.clear()
                    self._reject()
                break
            raw = bytes(self._buf[:idx])
            del self._buf[: idx + 1]
            parsed = self._parse_line(raw)
            if parsed is None:
                continue
            out.append(parsed)
            self.frames_ok += 1
            if isinstance(parsed, TimedQuaternionFrame):
                self.timed_frames_ok += 1
            elif isinstance(parsed, TimeSyncResponse):
                self.sync_frames_ok += 1
        self.last_events = tuple(out)
        return out

    feed_records = feed_events
    feed_protocol_events = feed_events

    def parse_line(self, raw: bytes | str) -> ProtocolEvent | None:
        if isinstance(raw, str):
            raw = raw.encode("utf-8")
        return self._parse_line(bytes(raw).rstrip(b"\r\n"))

    def _parse_line(self, raw: bytes) -> ProtocolEvent | None:
        if raw.endswith(b"\r"):
            raw = raw[:-1]
        if not raw.strip():
            self.frames_drop += 1
            return None
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            self._reject()
            return None
        parts = [part.strip() for part in text.strip().split(",")]
        if not parts:
            self._reject()
            return None
        prefix = parts[0]
        if prefix == "Q" and len(parts) == 5:
            return self._parse_quaternion(parts[1:])
        if prefix == "QT" and len(parts) == 8:
            boot_id = _parse_unsigned(parts[1], _U32_MAX)
            sequence = _parse_unsigned(parts[2], _U32_MAX)
            t_esp_us = _parse_signed(parts[3])
            q = self._parse_quaternion(parts[4:])
            if q is None:
                return None
            if boot_id is not None and sequence is not None and t_esp_us is not None:
                return TimedQuaternionFrame(boot_id, sequence, t_esp_us, q)
            self._reject()
            return None
        if prefix == "TSR" and len(parts) == 5:
            boot_id = _parse_unsigned(parts[1], _U32_MAX)
            sync_id = _parse_unsigned(parts[2], _U32_MAX)
            t2 = _parse_signed(parts[3])
            t3 = _parse_signed(parts[4])
            if (
                boot_id is not None
                and sync_id is not None
                and t2 is not None
                and t3 is not None
                and t2 >= 0
                and t3 >= 0
                and t3 >= t2
            ):
                return TimeSyncResponse(boot_id, sync_id, t2, t3)
            self._reject()
            return None
        self._reject()
        return None

    def _parse_quaternion(self, fields: list[str]) -> Quaternion | None:
        try:
            values = tuple(float(field) for field in fields)
        except (TypeError, ValueError):
            self._reject()
            return None
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            self._reject()
            return None
        mag = math.sqrt(sum(value * value for value in values))
        if mag < _MIN_NORM or mag > _MAX_NORM:
            self._reject()
            return None
        return tuple(value / mag for value in values)  # type: ignore[return-value]

    def _reject(self) -> None:
        self.frames_drop += 1
        self.parse_errors += 1


def _parse_unsigned(text: str, maximum: int) -> int | None:
    if not text or text[0] in "+-" or not text.isdigit():
        return None
    try:
        value = int(text, 10)
    except ValueError:
        return None
    return value if 0 <= value <= maximum else None


def _parse_signed(text: str) -> int | None:
    if not text:
        return None
    digits = text[1:] if text[0] in "+-" else text
    if not digits or not digits.isdigit():
        return None
    try:
        value = int(text, 10)
    except ValueError:
        return None
    return value if _I64_MIN <= value <= _I64_MAX else None
