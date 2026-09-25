from __future__ import annotations

import pytest

from imu_viewer.clock_sync import AffineClockMapper


def _add_exchange(
    mapper: AffineClockMapper,
    *,
    esp_mid_us: int,
    slope: float,
    offset: int,
    boot_id: int = 7,
    half_path_ns: int = 1_000_000,
    sync_id: int = 0,
) -> None:
    t2 = esp_mid_us - 5
    t3 = esp_mid_us + 5
    pc_mid = int(round(slope * esp_mid_us + offset))
    mapper.add_exchange(
        t1_pc_ns=pc_mid - half_path_ns,
        t2_esp_us=t2,
        t3_esp_us=t3,
        t4_pc_ns=pc_mid + half_path_ns,
        boot_id=boot_id,
        sync_id=sync_id,
    )


def test_affine_mapping_after_eight_exchanges():
    mapper = AffineClockMapper()
    slope = 1000.018
    offset = 5_000_000_000_000
    for i in range(8):
        _add_exchange(
            mapper,
            esp_mid_us=1_000_000 + i * 1_000_000,
            slope=slope,
            offset=offset,
            sync_id=i,
        )

    quality = mapper.quality()
    assert quality.ready is True
    assert quality.boot_id == 7
    assert quality.sample_count == 8
    assert quality.slope_ns_per_esp_us == pytest.approx(slope, abs=0.002)
    expected = int(round(slope * 12_345_678 + offset))
    assert mapper.map_esp_us(12_345_678, 7) == pytest.approx(expected, abs=10)


def test_high_rtt_outlier_is_not_used_for_mapping():
    mapper = AffineClockMapper(keep_fraction=0.5)
    slope = 999.992
    offset = 4_000_000_000_000
    for i in range(12):
        _add_exchange(
            mapper,
            esp_mid_us=2_000_000 + i * 500_000,
            slope=slope,
            offset=offset,
            sync_id=i,
        )

    # A high RTT sample whose midpoint is badly delayed must lose to the
    # lower-RTT half of the rolling window.
    esp = 9_000_000
    t2, t3 = esp - 5, esp + 5
    bad_mid = int(slope * esp + offset + 100_000_000)
    mapper.add_exchange(
        t1_pc_ns=bad_mid - 250_000_000,
        t2_esp_us=t2,
        t3_esp_us=t3,
        t4_pc_ns=bad_mid + 250_000_000,
        boot_id=7,
        sync_id=99,
    )

    assert mapper.map_esp_us(10_000_000, 7) == pytest.approx(
        slope * 10_000_000 + offset,
        abs=100,
    )
    assert mapper.quality().used_sample_count < mapper.quality().sample_count


def test_invalid_exchange_and_new_boot_clear_mapping():
    mapper = AffineClockMapper(min_samples=2)
    mapper.add_exchange(
        t1_pc_ns=2_000,
        t2_esp_us=10,
        t3_esp_us=11,
        t4_pc_ns=1_000,
        boot_id=1,
    )
    assert mapper.quality().invalid_count == 1
    assert mapper.map_esp_us(100, 1) is None

    _add_exchange(mapper, esp_mid_us=1000, slope=1000.0, offset=10_000, boot_id=1)
    _add_exchange(mapper, esp_mid_us=2000, slope=1000.0, offset=10_000, boot_id=1)
    assert mapper.map_esp_us(3000, 1) is not None

    _add_exchange(mapper, esp_mid_us=1000, slope=1000.0, offset=20_000, boot_id=2)
    assert mapper.quality().boot_id == 2
    assert mapper.map_esp_us(3000, 1) is None
    assert mapper.map_esp_us(3000, 2) is None
