from __future__ import annotations

import pytest

from imu_viewer.camera_time import (
    CameraClockDomain,
    CameraFrameTimestamp,
    CameraTimeAdapter,
    ExposureReference,
)


def test_pc_monotonic_start_timestamp_is_normalized_to_exposure_midpoint():
    adapter = CameraTimeAdapter()
    frame = adapter.convert(
        CameraFrameTimestamp(
            frame_id=3,
            timestamp=1_000_000,
            clock_domain=CameraClockDomain.PC_MONOTONIC_NS,
            exposure_reference=ExposureReference.START,
            duration_ns=20_000,
        )
    )
    assert frame is not None
    assert frame.t_camera_pc_ns == 1_010_000
    assert frame.exposure_reference is ExposureReference.MID
    assert frame.source_exposure_reference is ExposureReference.START


def test_wall_clock_needs_anchor_and_preserves_mapping_version():
    adapter = CameraTimeAdapter()
    source = CameraFrameTimestamp(
        frame_id=4,
        timestamp=2_000_500,
        clock_domain=CameraClockDomain.PC_WALL_NS,
        exposure_reference=ExposureReference.MID,
    )
    assert adapter.convert(source) is None
    anchor = adapter.add_wall_monotonic_anchor(2_000_000, 9_000_000)
    frame = adapter.convert(source)
    assert frame is not None
    assert frame.t_camera_pc_ns == 9_000_500
    assert frame.clock_map_version == anchor.version


def test_camera_ticks_are_not_treated_as_pc_time_without_mapping():
    adapter = CameraTimeAdapter()
    source = CameraFrameTimestamp(
        frame_id=5,
        timestamp=100,
        clock_domain=CameraClockDomain.CAMERA_TICKS,
        exposure_reference=ExposureReference.MID,
    )
    assert adapter.convert(source) is None
    adapter.camera_mapper.set_mapping(1_000.0, 10_000.0)
    frame = adapter.convert(source)
    assert frame is not None
    assert frame.t_camera_pc_ns == 110_000


def test_start_or_end_requires_exposure_duration():
    adapter = CameraTimeAdapter()
    with pytest.raises(ValueError, match="duration_ns"):
        adapter.convert(
            CameraFrameTimestamp(
                frame_id=1,
                timestamp=100,
                clock_domain=CameraClockDomain.PC_MONOTONIC_NS,
                exposure_reference=ExposureReference.END,
            )
        )
