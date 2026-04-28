"""Unit tests for the pure-Python UBX protocol module."""

import struct

import pytest

from ubx_parser.ubx_protocol import (
    CLS_NAV,
    ID_NAV_PVT,
    SYNC1,
    SYNC2,
    checksum,
    decode_nav_pvt,
    encode_frame,
    iter_frames,
)


def _build_nav_pvt_payload(*, lat_deg=37.4220, lon_deg=-122.0841,
                           h_msl_m=10.0, fix_type=3, num_sv=12,
                           i_tow_ms=123_456_000):
    return struct.pack(
        '<IHBBBBBBIiBBBBiiiiIIiiiiiIIH',
        i_tow_ms,
        2026, 4, 15,
        17, 42, 45, 0x07,
        50, 0,
        fix_type, 0x01, 0x00, num_sv,
        int(round(lon_deg * 1e7)),
        int(round(lat_deg * 1e7)),
        int(round(h_msl_m * 1000)) + 25000,  # ellipsoid height ~ MSL + geoid
        int(round(h_msl_m * 1000)),
        1500, 2500,
        100, -200, 300, 400,
        9000000, 250, 1500000,
        180,
    )


def test_checksum_round_trip():
    payload = b'\x01\x02\x03\x04'
    ck_a, ck_b = checksum(0x05, 0x06, payload)
    frame = bytes([SYNC1, SYNC2, 0x05, 0x06, 4, 0]) + payload + bytes([ck_a, ck_b])
    decoded = list(iter_frames(frame))
    assert len(decoded) == 1
    assert decoded[0].msg_class == 0x05
    assert decoded[0].msg_id == 0x06
    assert decoded[0].payload == payload


def test_iter_frames_skips_garbage_between_frames():
    payload = b'\xaa\xbb'
    good = encode_frame(0x10, 0x20, payload)
    junk = b'\x00\x11\x22\x33\xb5\x00\xff'
    stream = junk + good + junk + good
    frames = list(iter_frames(stream))
    assert [f.payload for f in frames] == [payload, payload]


def test_iter_frames_handles_truncated_tail():
    good = encode_frame(0x10, 0x20, b'\x00\x01\x02')
    truncated = good[:-1]
    frames = list(iter_frames(good + truncated))
    assert len(frames) == 1


def test_iter_frames_rejects_oversized_length():
    fake = bytes([SYNC1, SYNC2, 0x10, 0x20, 0xFF, 0xFF])
    assert list(iter_frames(fake + b'\x00' * 16)) == []


def test_decode_nav_pvt_round_trip():
    payload = _build_nav_pvt_payload(lat_deg=37.4220, lon_deg=-122.0841,
                                     h_msl_m=10.0, fix_type=3, num_sv=12)
    pvt = decode_nav_pvt(payload)
    assert pvt is not None
    assert pvt.fix_type == 3
    assert pvt.num_sv == 12
    assert pvt.lat_deg == pytest.approx(37.4220, abs=1e-6)
    assert pvt.lon_deg == pytest.approx(-122.0841, abs=1e-6)
    assert pvt.h_msl_m == pytest.approx(10.0, abs=1e-3)
    assert pvt.year == 2026
    assert pvt.month == 4
    assert pvt.day == 15


def test_decode_nav_pvt_rejects_short_payload():
    assert decode_nav_pvt(b'\x00' * 10) is None


def test_full_frame_decode_via_iter():
    payload = _build_nav_pvt_payload()
    frame = encode_frame(CLS_NAV, ID_NAV_PVT, payload)
    decoded = list(iter_frames(b'\x00\x01' + frame + b'\xff'))
    assert len(decoded) == 1
    assert decoded[0].key == (CLS_NAV, ID_NAV_PVT)
    pvt = decode_nav_pvt(decoded[0].payload)
    assert pvt is not None
    assert pvt.fix_type == 3
