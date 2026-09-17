"""SH-01 text protocol: byte buffer, split on newline, parse Q,w,x,y,z."""

from __future__ import annotations

import math

Quaternion = tuple[float, float, float, float]

_MIN_NORM = 1e-6
_MAX_NORM = 2.0


class ProtocolDecoder:
    """Append BLE notify fragments; emit complete valid quaternions."""

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.frames_ok = 0
        self.frames_drop = 0
        self.parse_errors = 0

    def feed(self, data: bytes) -> list[Quaternion]:
        """Append bytes; return this batch of successfully parsed (w,x,y,z).

        Incomplete lines stay buffered. Illegal complete frames are dropped
        and counted; the buffer keeps accepting later data.
        """
        if not data:
            return []
        self._buf.extend(data)
        out: list[Quaternion] = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                if len(self._buf) > self._max_buffer:
                    self._buf.clear()
                    self.frames_drop += 1
                    self.parse_errors += 1
                break
            raw = bytes(self._buf[:idx])
            del self._buf[: idx + 1]
            parsed = self._parse_line(raw)
            if parsed is None:
                continue
            out.append(parsed)
            self.frames_ok += 1
        return out

    def _parse_line(self, raw: bytes) -> Quaternion | None:
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
        parts = [p.strip() for p in text.strip().split(",")]
        if len(parts) != 5 or parts[0] != "Q":
            self._reject()
            return None
        try:
            w, x, y, z = (float(p) for p in parts[1:])
        except ValueError:
            self._reject()
            return None
        if not all(math.isfinite(v) for v in (w, x, y, z)):
            self._reject()
            return None
        mag = math.sqrt(w * w + x * x + y * y + z * z)
        if mag < _MIN_NORM or mag > _MAX_NORM:
            self._reject()
            return None
        return (w / mag, x / mag, y / mag, z / mag)

    def _reject(self) -> None:
        self.frames_drop += 1
        self.parse_errors += 1
