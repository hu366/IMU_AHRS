from __future__ import annotations

from imu_viewer.protocol import ProtocolDecoder, TimeSyncResponse, TimedQuaternionFrame


def test_qt_and_tsr_survive_fragmentation_and_interleaving():
    decoder = ProtocolDecoder()
    assert decoder.feed_events(b"QT,39182744,1523,987654") == []
    events = decoder.feed_events(
        b"321,0.9981,0.0123,-0.0310,0.0512\n"
        b"TSR,39182744,8,987654400,987654430\n"
        b"Q,1,0,0,0\n"
    )
    assert len(events) == 3
    assert isinstance(events[0], TimedQuaternionFrame)
    assert events[0].boot_id == 39182744
    assert events[0].sequence == 1523
    assert events[0].t_data_ready_us == 987654321
    assert isinstance(events[1], TimeSyncResponse)
    assert events[1].sync_id == 8
    assert events[1].t2_esp_us == 987654400
    assert decoder.frames_ok == 3


def test_invalid_timed_and_sync_fields_are_counted_once_per_line():
    decoder = ProtocolDecoder()
    assert decoder.feed_events(
        b"QT,1,2,3,nan,0,0,0\n"
        b"TSR,1,2,20,10\n"
        b"TSR,1,,10,11\n"
    ) == []
    assert decoder.frames_drop == 3
    assert decoder.parse_errors == 3
