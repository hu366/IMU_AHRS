from __future__ import annotations

import math

import pytest

from imu_viewer.alignment import (
    AlignedFrame,
    CameraFrameAligner,
    TimedPoseBuffer,
    TimedQuaternion,
    UnmatchedFrame,
)
from imu_viewer.camera_time import CameraFrameTime, ExposureReference
from imu_viewer.clock_sync import AffineClockMapper
from imu_viewer.protocol import TimedQuaternionFrame


def _camera(frame_id: int, t_pc_ns: int) -> CameraFrameTime:
    return CameraFrameTime(
        frame_id=frame_id,
        t_camera_pc_ns=t_pc_ns,
        exposure_reference=ExposureReference.MID,
        source_timestamp=t_pc_ns,
    )


def _pose(sequence: int, t_pc_ns: int, q: tuple[float, float, float, float]) -> TimedQuaternion:
    return TimedQuaternion(
        boot_id=1,
        sequence=sequence,
        t_esp_us=t_pc_ns // 1000,
        t_pc_ns=t_pc_ns,
        quaternion_wxyz=q,
        clock_map_version=4,
    )


def test_slerp_midpoint_uses_wxyz_to_xyzw_conversion_once():
    buffer = TimedPoseBuffer()
    aligner = CameraFrameAligner(pose_buffer=buffer)
    aligner.reset(1)
    buffer.add(_pose(10, 100_000_000, (1.0, 0.0, 0.0, 0.0)))
    buffer.add(_pose(11, 110_000_000, (0.0, 0.0, 0.0, 1.0)))

    result = aligner.align(_camera(7, 105_000_000))
    assert isinstance(result, AlignedFrame)
    assert result.imu_sequence_before == 10
    assert result.imu_sequence_after == 11
    assert result.interpolation_alpha == pytest.approx(0.5)
    assert result.time_gap_ns == 10_000_000
    assert result.quaternion_wxyz[0] == pytest.approx(math.sqrt(0.5), abs=1e-7)
    assert result.quaternion_wxyz[3] == pytest.approx(math.sqrt(0.5), abs=1e-7)


def test_missing_bracket_and_large_interval_are_explicitly_unmatched():
    buffer = TimedPoseBuffer()
    aligner = CameraFrameAligner(
        pose_buffer=buffer,
        max_interpolation_gap_ns=30_000_000,
    )
    aligner.reset(1)
    buffer.add(_pose(1, 100_000_000, (1.0, 0.0, 0.0, 0.0)))
    assert isinstance(aligner.align(_camera(1, 100_000_000)), UnmatchedFrame)

    buffer.add(_pose(2, 200_000_000, (1.0, 0.0, 0.0, 0.0)))
    result = aligner.align(_camera(2, 150_000_000))
    assert isinstance(result, UnmatchedFrame)
    assert result.reason == "imu_interval_too_large"


def test_out_of_order_input_is_time_sorted_for_bracketing():
    buffer = TimedPoseBuffer()
    aligner = CameraFrameAligner(pose_buffer=buffer)
    aligner.reset(1)
    buffer.add(_pose(2, 120_000_000, (1.0, 0.0, 0.0, 0.0)))
    buffer.add(_pose(1, 100_000_000, (1.0, 0.0, 0.0, 0.0)))
    result = aligner.align(_camera(1, 110_000_000))
    assert isinstance(result, AlignedFrame)
    assert (result.imu_sequence_before, result.imu_sequence_after) == (1, 2)


def test_aligned_frame_carries_the_clock_quality_snapshot():
    mapper = AffineClockMapper(min_samples=2)
    for esp_us in (100_000, 110_000):
        pc_ns = esp_us * 1_000
        mapper.add_exchange(
            t1_pc_ns=pc_ns - 1_000,
            t2_esp_us=esp_us,
            t3_esp_us=esp_us,
            t4_pc_ns=pc_ns + 1_000,
            boot_id=1,
        )

    aligner = CameraFrameAligner()
    aligner.ingest_timed_quaternion(
        TimedQuaternionFrame(1, 10, 100_000, (1.0, 0.0, 0.0, 0.0)), mapper
    )
    aligner.ingest_timed_quaternion(
        TimedQuaternionFrame(1, 11, 110_000, (0.0, 0.0, 0.0, 1.0)), mapper
    )

    result = aligner.align(_camera(9, 105_000_000))
    assert isinstance(result, AlignedFrame)
    assert result.clock_quality is not None
    assert result.clock_quality.ready is True
    assert result.clock_quality.sample_count == 2
    assert result.clock_map_version == result.clock_quality.map_version
