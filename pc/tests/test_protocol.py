from __future__ import annotations

from pathlib import Path

import pytest

from imu_viewer.protocol import ProtocolDecoder

IDENTITY_LINE = b"Q,1.0000,0.0000,0.0000,0.0000\n"
SAMPLE_LINE = b"Q,0.9981,0.0123,-0.0310,0.0512\n"
DATA_FILE = Path(__file__).parent / "data" / "sample_frames.txt"


def _approx_q(actual, expected, rel=1e-4, abs_=1e-4):
    assert actual == pytest.approx(expected, rel=rel, abs=abs_)


def test_single_identity_frame():
    dec = ProtocolDecoder()
    frames = dec.feed(IDENTITY_LINE)
    assert len(frames) == 1
    _approx_q(frames[0], (1.0, 0.0, 0.0, 0.0))
    assert dec.frames_ok == 1
    assert dec.frames_drop == 0


def test_standard_sample_vector():
    dec = ProtocolDecoder()
    frames = dec.feed(SAMPLE_LINE)
    assert len(frames) == 1
    w, x, y, z = frames[0]
    mag = (w * w + x * x + y * y + z * z) ** 0.5
    assert mag == pytest.approx(1.0, abs=1e-9)
    _approx_q((w, x, y, z), (0.9981, 0.0123, -0.0310, 0.0512), rel=1e-3, abs_=1e-3)


def test_sticky_two_frames_in_one_chunk():
    dec = ProtocolDecoder()
    frames = dec.feed(IDENTITY_LINE + SAMPLE_LINE)
    assert len(frames) == 2
    _approx_q(frames[0], (1.0, 0.0, 0.0, 0.0))
    assert dec.frames_ok == 2


def test_fragment_then_complete():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,1.0") == []
    assert dec.frames_ok == 0
    frames = dec.feed(b"000,0.0000,0.0000,0.0000\n")
    assert len(frames) == 1
    _approx_q(frames[0], (1.0, 0.0, 0.0, 0.0))
    assert dec.frames_ok == 1


def test_fragment_split_inside_a_field():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,1.0000,0.00") == []
    frames = dec.feed(b"00,0.0000,0.0000\n")
    assert len(frames) == 1
    _approx_q(frames[0], (1.0, 0.0, 0.0, 0.0))


def test_crlf_is_stripped_but_not_required():
    dec = ProtocolDecoder()
    frames = dec.feed(b"Q,1.0000,0.0000,0.0000,0.0000\r\n")
    assert len(frames) == 1
    _approx_q(frames[0], (1.0, 0.0, 0.0, 0.0))


def test_empty_line_dropped():
    dec = ProtocolDecoder()
    frames = dec.feed(b"\n\n" + IDENTITY_LINE)
    assert len(frames) == 1
    assert dec.frames_drop == 2
    assert dec.parse_errors == 0


def test_missing_fields():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,1.0,0,0\n") == []
    assert dec.frames_drop == 1
    assert dec.parse_errors == 1


def test_non_numeric():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,abc,0,0,0\n") == []
    assert dec.frames_drop == 1
    assert dec.parse_errors == 1


def test_nan_and_inf_dropped():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,nan,0,0,0\n") == []
    assert dec.feed(b"Q,inf,0,0,0\n") == []
    assert dec.feed(b"Q,1,0,0,inf\n") == []
    assert dec.frames_ok == 0
    assert dec.frames_drop == 3
    assert dec.parse_errors == 3


def test_zero_norm_dropped():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,0,0,0,0\n") == []
    assert dec.frames_drop == 1
    assert dec.parse_errors == 1


def test_norm_below_1e_6_dropped():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,1e-7,0,0,0\n") == []
    assert dec.frames_drop == 1
    assert dec.parse_errors == 1


def test_norm_above_two_dropped():
    dec = ProtocolDecoder()
    assert dec.feed(b"Q,3,0,0,0\n") == []
    assert dec.frames_drop == 1


def test_error_does_not_break_later_frames():
    dec = ProtocolDecoder()
    blob = (
        b"\n"
        b"Q,abc,0,0,0\n"
        b"Q,nan,0,0,0\n"
        b"Q,1.0,0,0\n"
        b"Q,0,0,0,0\n"
        + IDENTITY_LINE
        + SAMPLE_LINE
    )
    frames = dec.feed(blob)
    assert len(frames) == 2
    _approx_q(frames[0], (1.0, 0.0, 0.0, 0.0))
    assert dec.frames_ok == 2
    assert dec.frames_drop >= 4


def test_wrong_prefix_dropped():
    dec = ProtocolDecoder()
    assert dec.feed(b"P,1.0000,0.0000,0.0000,0.0000\n") == []
    assert dec.frames_drop == 1


def test_recorded_sample_file():
    dec = ProtocolDecoder()
    raw = DATA_FILE.read_bytes()
    if not raw.endswith(b"\n"):
        raw += b"\n"
    frames = dec.feed(raw)
    assert len(frames) == 3
    _approx_q(frames[0], (1.0, 0.0, 0.0, 0.0))
    _approx_q(frames[2], (1.0, 0.0, 0.0, 0.0))
    assert dec.frames_drop >= 3
    assert dec.frames_ok == 3


def test_buffer_overflow_without_newline_is_dropped():
    dec = ProtocolDecoder(max_buffer=32)
    assert dec.feed(b"Q," + b"1" * 64) == []
    assert dec.parse_errors == 1
    frames = dec.feed(IDENTITY_LINE)
    assert len(frames) == 1
