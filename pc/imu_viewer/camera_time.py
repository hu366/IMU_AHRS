"""Camera timestamp contracts and conversion to the PC monotonic clock."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum


class CameraClockDomain(StrEnum):
    PC_MONOTONIC_NS = "pc_monotonic_ns"
    PC_WALL_NS = "pc_wall_ns"
    CAMERA_TICKS = "camera_ticks"


class ExposureReference(StrEnum):
    START = "start"
    MID = "mid"
    END = "end"


@dataclass(frozen=True)
class CameraFrameTimestamp:
    frame_id: int
    timestamp: int
    clock_domain: CameraClockDomain
    exposure_reference: ExposureReference
    duration_ns: int | None = None


@dataclass(frozen=True)
class CameraFrameTime:
    frame_id: int
    t_camera_pc_ns: int
    exposure_reference: ExposureReference
    source_timestamp: int
    source_clock_domain: CameraClockDomain | None = None
    source_exposure_reference: ExposureReference | None = None
    clock_map_version: int = 0


@dataclass(frozen=True)
class WallMonotonicAnchor:
    wall_ns: int
    monotonic_ns: int
    version: int


class WallToMonotonicMapper:
    """Piecewise wall-clock to monotonic-clock conversion.

    Each anchor is captured in the same PC process. New anchors create a new
    mapping version, so a later system-wall-clock correction cannot silently
    change timestamps already emitted for previous frames.
    """

    def __init__(self, max_anchors: int = 32) -> None:
        if max_anchors < 1:
            raise ValueError("max_anchors must be positive")
        self._max_anchors = max_anchors
        self._anchors: list[WallMonotonicAnchor] = []
        self._version = 0

    @property
    def version(self) -> int:
        return self._version

    @property
    def anchors(self) -> tuple[WallMonotonicAnchor, ...]:
        return tuple(self._anchors)

    def add_anchor(self, wall_ns: int, monotonic_ns: int) -> WallMonotonicAnchor:
        wall = int(wall_ns)
        monotonic = int(monotonic_ns)
        if wall < 0 or monotonic < 0:
            raise ValueError("wall_ns and monotonic_ns must be non-negative")
        self._version += 1
        anchor = WallMonotonicAnchor(wall, monotonic, self._version)
        self._anchors.append(anchor)
        if len(self._anchors) > self._max_anchors:
            del self._anchors[: len(self._anchors) - self._max_anchors]
        return anchor

    record_anchor = add_anchor

    def map_wall_ns(self, t_camera_wall_ns: int) -> tuple[int, int] | None:
        if not self._anchors:
            return None
        wall = int(t_camera_wall_ns)
        anchor = min(self._anchors, key=lambda item: abs(wall - item.wall_ns))
        return anchor.monotonic_ns + (wall - anchor.wall_ns), anchor.version


class CameraClockMapper:
    """Optional affine mapper for a camera-owned hardware tick clock."""

    def __init__(self) -> None:
        self._a: float | None = None
        self._b: float | None = None
        self._version = 0

    @property
    def ready(self) -> bool:
        return self._a is not None and self._b is not None

    @property
    def version(self) -> int:
        return self._version

    def set_affine_mapping(self, ticks_to_pc_ns: float, offset_pc_ns: float) -> None:
        if not math.isfinite(ticks_to_pc_ns) or not math.isfinite(offset_pc_ns):
            raise ValueError("camera clock mapping must be finite")
        self._a = float(ticks_to_pc_ns)
        self._b = float(offset_pc_ns)
        self._version += 1

    set_mapping = set_affine_mapping

    def map_ticks(self, camera_ticks: int) -> tuple[int, int] | None:
        if self._a is None or self._b is None:
            return None
        value = self._a * int(camera_ticks) + self._b
        return (int(round(value)), self._version) if math.isfinite(value) else None


class CameraTimeAdapter:
    """Convert a camera SDK timestamp into the PC monotonic time domain."""

    def __init__(
        self,
        *,
        wall_mapper: WallToMonotonicMapper | None = None,
        camera_mapper: CameraClockMapper | None = None,
    ) -> None:
        self.wall_mapper = wall_mapper or WallToMonotonicMapper()
        self.camera_mapper = camera_mapper or CameraClockMapper()
        self._last_camera_time_ns: int | None = None

    def add_wall_monotonic_anchor(self, wall_ns: int, monotonic_ns: int) -> WallMonotonicAnchor:
        return self.wall_mapper.add_anchor(wall_ns, monotonic_ns)

    def convert(self, frame: CameraFrameTimestamp) -> CameraFrameTime | None:
        """Return an exposure-midpoint time or None while a mapper is unavailable."""
        timestamp = int(frame.timestamp)
        if timestamp < 0:
            raise ValueError("camera timestamp must be non-negative")
        if frame.duration_ns is not None and int(frame.duration_ns) < 0:
            raise ValueError("duration_ns must be non-negative")
        if frame.clock_domain is CameraClockDomain.PC_MONOTONIC_NS:
            t_pc_ns, version = timestamp, 0
        elif frame.clock_domain is CameraClockDomain.PC_WALL_NS:
            mapped = self.wall_mapper.map_wall_ns(timestamp)
            if mapped is None:
                return None
            t_pc_ns, version = mapped
        elif frame.clock_domain is CameraClockDomain.CAMERA_TICKS:
            mapped = self.camera_mapper.map_ticks(timestamp)
            if mapped is None:
                return None
            t_pc_ns, version = mapped
        else:
            raise ValueError(f"unsupported camera clock domain: {frame.clock_domain!r}")

        t_mid_ns = _to_exposure_midpoint(
            t_pc_ns,
            frame.exposure_reference,
            frame.duration_ns,
        )
        if self._last_camera_time_ns is not None and t_mid_ns < self._last_camera_time_ns:
            raise ValueError("camera exposure time moved backwards")
        self._last_camera_time_ns = t_mid_ns
        return CameraFrameTime(
            frame_id=int(frame.frame_id),
            t_camera_pc_ns=t_mid_ns,
            exposure_reference=ExposureReference.MID,
            source_timestamp=timestamp,
            source_clock_domain=frame.clock_domain,
            source_exposure_reference=frame.exposure_reference,
            clock_map_version=version,
        )

    adapt = convert
    to_pc_monotonic = convert


CameraTimestampAdapter = CameraTimeAdapter


def _to_exposure_midpoint(
    timestamp_ns: int,
    reference: ExposureReference,
    duration_ns: int | None,
) -> int:
    if reference is ExposureReference.MID:
        return timestamp_ns
    if duration_ns is None:
        raise ValueError("duration_ns is required for START or END exposure timestamps")
    half = int(duration_ns) // 2
    return timestamp_ns + half if reference is ExposureReference.START else timestamp_ns - half
