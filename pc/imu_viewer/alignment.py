"""Historical IMU pose buffering and camera-exposure pose alignment."""

from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass
from typing import TypeAlias

from imu_viewer.camera_time import CameraFrameTime, CameraFrameTimestamp, CameraTimeAdapter
from imu_viewer.clock_sync import AffineClockMapper, ClockMapQuality
from imu_viewer.protocol import Quaternion, TimedQuaternionFrame


@dataclass(frozen=True)
class TimedQuaternion:
    boot_id: int
    sequence: int
    t_esp_us: int
    t_pc_ns: int
    quaternion_wxyz: Quaternion
    clock_map_version: int = 0


@dataclass(frozen=True)
class AlignedFrame:
    frame_id: int
    t_camera_pc_ns: int
    quaternion_wxyz: Quaternion
    imu_sequence_before: int
    imu_sequence_after: int
    interpolation_alpha: float
    time_gap_ns: int
    boot_id: int | None = None
    clock_map_version: int = 0
    clock_quality: ClockMapQuality | None = None
    status: str = "aligned"


@dataclass(frozen=True)
class UnmatchedFrame:
    frame_id: int
    t_camera_pc_ns: int
    status: str
    reason: str
    boot_id: int | None = None

    @property
    def matched(self) -> bool:
        return False


AlignmentResult: TypeAlias = AlignedFrame | UnmatchedFrame


class TimedPoseBuffer:
    """A bounded, time-sorted buffer of mapped IMU quaternion samples."""

    def __init__(
        self,
        *,
        max_samples: int = 2048,
        history_ns: int | None = 10_000_000_000,
    ) -> None:
        if max_samples < 2:
            raise ValueError("max_samples must be at least 2")
        if history_ns is not None and history_ns <= 0:
            raise ValueError("history_ns must be positive or None")
        self.max_samples = max_samples
        self.history_ns = history_ns
        self._poses: list[TimedQuaternion] = []
        self._lock = threading.RLock()
        self.dropped_invalid = 0
        self.dropped_out_of_order = 0

    def __len__(self) -> int:
        with self._lock:
            return len(self._poses)

    def snapshot(self) -> tuple[TimedQuaternion, ...]:
        with self._lock:
            return tuple(self._poses)

    def clear(self) -> None:
        with self._lock:
            self._poses.clear()

    def add(self, pose: TimedQuaternion) -> bool:
        """Insert a pose by PC time; returns false when its quaternion is invalid."""
        if not _quaternion_is_valid(pose.quaternion_wxyz) or pose.t_pc_ns < 0:
            self.dropped_invalid += 1
            return False
        with self._lock:
            times = [item.t_pc_ns for item in self._poses]
            index = bisect.bisect_left(times, pose.t_pc_ns)
            if index < len(self._poses) and self._poses[index].t_pc_ns == pose.t_pc_ns:
                # Replace equal-time samples deterministically with the newest input.
                self._poses[index] = pose
            else:
                self._poses.insert(index, pose)
                if index != len(self._poses) - 1:
                    self.dropped_out_of_order += 1
            self._trim_locked()
        return True

    append = add

    def bracket(self, t_pc_ns: int) -> tuple[TimedQuaternion, TimedQuaternion] | None:
        """Return the samples directly around a PC monotonic camera time."""
        with self._lock:
            if len(self._poses) < 2:
                return None
            times = [item.t_pc_ns for item in self._poses]
            after_index = bisect.bisect_left(times, int(t_pc_ns))
            if after_index < len(self._poses) and self._poses[after_index].t_pc_ns == t_pc_ns:
                if after_index + 1 < len(self._poses):
                    return self._poses[after_index], self._poses[after_index + 1]
                if after_index > 0:
                    return self._poses[after_index - 1], self._poses[after_index]
                return None
            if after_index == 0 or after_index >= len(self._poses):
                return None
            return self._poses[after_index - 1], self._poses[after_index]

    find_bracket = bracket

    def replace_all(self, poses: list[TimedQuaternion]) -> None:
        """Replace contents after a clock-map re-projection."""
        with self._lock:
            valid = [item for item in poses if _quaternion_is_valid(item.quaternion_wxyz)]
            valid.sort(key=lambda item: item.t_pc_ns)
            self._poses = valid
            self._trim_locked()

    def _trim_locked(self) -> None:
        if self.history_ns is not None and self._poses:
            cutoff = self._poses[-1].t_pc_ns - self.history_ns
            first = bisect.bisect_left([item.t_pc_ns for item in self._poses], cutoff)
            if first:
                del self._poses[:first]
        if len(self._poses) > self.max_samples:
            del self._poses[: len(self._poses) - self.max_samples]


class CameraFrameAligner:
    """Map QT samples into PC time and interpolate them at camera exposure time."""

    def __init__(
        self,
        *,
        pose_buffer: TimedPoseBuffer | None = None,
        max_interpolation_gap_ns: int = 30_000_000,
        pending_limit: int = 4096,
        camera_time_adapter: CameraTimeAdapter | None = None,
    ) -> None:
        if max_interpolation_gap_ns <= 0:
            raise ValueError("max_interpolation_gap_ns must be positive")
        self.pose_buffer = pose_buffer if pose_buffer is not None else TimedPoseBuffer()
        self.max_interpolation_gap_ns = int(max_interpolation_gap_ns)
        self.pending_limit = int(pending_limit)
        self.camera_time_adapter = camera_time_adapter
        self._boot_id: int | None = None
        self._clock_quality: ClockMapQuality | None = None
        self._pending: list[TimedQuaternionFrame] = []
        self._lock = threading.RLock()
        self.mapped_pose_count = 0
        self.unmapped_pose_count = 0

    @property
    def boot_id(self) -> int | None:
        return self._boot_id

    @property
    def clock_quality(self) -> ClockMapQuality | None:
        with self._lock:
            return self._clock_quality

    def reset(self, boot_id: int | None = None) -> None:
        with self._lock:
            self._boot_id = boot_id
            self._clock_quality = None
            self._pending.clear()
            self.pose_buffer.clear()

    clear = reset

    def ingest_timed_quaternion(
        self,
        frame: TimedQuaternionFrame,
        mapper: AffineClockMapper,
    ) -> TimedQuaternion | None:
        """Store a QT and map it without blocking the BLE callback.

        The clock map changes only when a TSR arrives (normally once per
        second). Re-projecting the complete history for every 100 Hz QT frame
        can delay the BLE event loop and corrupt the PC t4 timestamp.
        """
        with self._lock:
            if self._boot_id != frame.boot_id:
                self.reset(frame.boot_id)

            quality = mapper.quality()
            self._clock_quality = quality
            mapped_t_pc_ns = mapper.map_esp_us(frame.t_esp_us, frame.boot_id)
            if mapped_t_pc_ns is None:
                self._pending.append(frame)
                if len(self._pending) > self.pending_limit:
                    del self._pending[: len(self._pending) - self.pending_limit]
                    self.unmapped_pose_count += 1
                return None

            pose = TimedQuaternion(
                boot_id=frame.boot_id,
                sequence=frame.sequence,
                t_esp_us=frame.t_esp_us,
                t_pc_ns=mapped_t_pc_ns,
                quaternion_wxyz=frame.quaternion_wxyz,
                clock_map_version=quality.map_version,
            )
            if self.pose_buffer.add(pose):
                self.mapped_pose_count += 1
                return pose
            return None

    ingest_pose = ingest_timed_quaternion

    def refresh_clock_map(self, mapper: AffineClockMapper) -> None:
        """Re-project buffered raw poses whenever the affine map changes."""
        with self._lock:
            quality = mapper.quality()
            self._clock_quality = quality
            map_boot_id = mapper.boot_id
            if map_boot_id is None:
                return
            if self._boot_id is None:
                self._boot_id = map_boot_id
            if self._boot_id != map_boot_id:
                # A QT frame has already identified a newer boot than the
                # mapper's last exchange. Keep its raw samples pending until
                # TSQ/TSR establishes the matching new clock map.
                return
            if not quality.ready:
                return
            projected: list[TimedQuaternion] = []
            remaining: list[TimedQuaternionFrame] = []
            for frame in self._pending:
                t_pc_ns = mapper.map_esp_us(frame.t_esp_us, frame.boot_id)
                if t_pc_ns is None:
                    remaining.append(frame)
                    continue
                projected.append(
                    TimedQuaternion(
                        boot_id=frame.boot_id,
                        sequence=frame.sequence,
                        t_esp_us=frame.t_esp_us,
                        t_pc_ns=t_pc_ns,
                        quaternion_wxyz=frame.quaternion_wxyz,
                        clock_map_version=quality.map_version,
                    )
                )
            self._pending = remaining
            if projected:
                existing = [
                    pose
                    for pose in self.pose_buffer.snapshot()
                    if pose.boot_id == map_boot_id
                ]
                existing_by_seq = {pose.sequence: pose for pose in existing}
                for pose in projected:
                    existing_by_seq[pose.sequence] = pose
                self.pose_buffer.replace_all(list(existing_by_seq.values()))
                self.mapped_pose_count += len(projected)

            # Re-project every retained sample under the newest mapping. This
            # makes clock-map versions explicit and never mixes coordinates.
            reprojection: list[TimedQuaternion] = []
            for pose in self.pose_buffer.snapshot():
                mapped = mapper.map_esp_us(pose.t_esp_us, pose.boot_id)
                if mapped is not None:
                    reprojection.append(
                        TimedQuaternion(
                            boot_id=pose.boot_id,
                            sequence=pose.sequence,
                            t_esp_us=pose.t_esp_us,
                            t_pc_ns=mapped,
                            quaternion_wxyz=pose.quaternion_wxyz,
                            clock_map_version=quality.map_version,
                        )
                    )
            self.pose_buffer.replace_all(reprojection)

    reproject = refresh_clock_map

    def align(self, frame: CameraFrameTime) -> AlignmentResult:
        """Interpolate an IMU orientation at a camera exposure midpoint."""
        with self._lock:
            if self._boot_id is None:
                return UnmatchedFrame(
                    frame.frame_id,
                    frame.t_camera_pc_ns,
                    "unmatched",
                    "clock_mapping_not_ready",
                )
            bracket = self.pose_buffer.bracket(frame.t_camera_pc_ns)
            if bracket is None:
                return UnmatchedFrame(
                    frame.frame_id,
                    frame.t_camera_pc_ns,
                    "unmatched",
                    "no_imu_bracket",
                    self._boot_id,
                )
            before, after = bracket
            if before.boot_id != after.boot_id:
                return UnmatchedFrame(
                    frame.frame_id,
                    frame.t_camera_pc_ns,
                    "unmatched",
                    "cross_boot_bracket",
                    self._boot_id,
                )
            gap = after.t_pc_ns - before.t_pc_ns
            if gap <= 0:
                return UnmatchedFrame(
                    frame.frame_id,
                    frame.t_camera_pc_ns,
                    "unmatched",
                    "non_increasing_imu_time",
                    self._boot_id,
                )
            if gap > self.max_interpolation_gap_ns:
                return UnmatchedFrame(
                    frame.frame_id,
                    frame.t_camera_pc_ns,
                    "unmatched",
                    "imu_interval_too_large",
                    self._boot_id,
                )
            alpha = (frame.t_camera_pc_ns - before.t_pc_ns) / gap
            if alpha < 0.0 or alpha > 1.0:
                return UnmatchedFrame(
                    frame.frame_id,
                    frame.t_camera_pc_ns,
                    "unmatched",
                    "camera_outside_imu_bracket",
                    self._boot_id,
                )
            quaternion = _slerp_wxyz(before.quaternion_wxyz, after.quaternion_wxyz, alpha)
            return AlignedFrame(
                frame_id=frame.frame_id,
                t_camera_pc_ns=frame.t_camera_pc_ns,
                quaternion_wxyz=quaternion,
                imu_sequence_before=before.sequence,
                imu_sequence_after=after.sequence,
                interpolation_alpha=alpha,
                time_gap_ns=gap,
                boot_id=before.boot_id,
                clock_map_version=max(before.clock_map_version, after.clock_map_version),
                clock_quality=self._clock_quality,
            )

    align_frame = align
    submit_camera_frame = align

    def align_camera_timestamp(self, frame: CameraFrameTimestamp) -> AlignmentResult:
        """Adapt a camera SDK timestamp, then align it at the exposure midpoint."""
        if self.camera_time_adapter is None:
            return UnmatchedFrame(
                frame.frame_id,
                0,
                "unmatched",
                "camera_time_adapter_not_configured",
                self._boot_id,
            )
        converted = self.camera_time_adapter.convert(frame)
        if converted is None:
            return UnmatchedFrame(
                frame.frame_id,
                0,
                "unmatched",
                "camera_clock_mapping_not_ready",
                self._boot_id,
            )
        return self.align(converted)


def _quaternion_is_valid(quaternion: Quaternion) -> bool:
    values = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    return len(values) == 4 and all(math.isfinite(value) for value in values) and norm > 1e-8


def _slerp_wxyz(q0: Quaternion, q1: Quaternion, alpha: float) -> Quaternion:
    """The only wxyz to scipy xyzw conversion point in the PC code."""
    from scipy.spatial.transform import Rotation, Slerp

    rotations = Rotation.from_quat(
        [
            [q0[1], q0[2], q0[3], q0[0]],
            [q1[1], q1[2], q1[3], q1[0]],
        ]
    )
    xyzw = Slerp([0.0, 1.0], rotations)([float(alpha)]).as_quat()[0]
    return (float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2]))
