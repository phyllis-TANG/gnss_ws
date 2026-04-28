"""UBX binary protocol parsing utilities.

The UBX frame layout is:

    [0xB5][0x62][class:U1][id:U1][len:U2 LE][payload:len bytes][ck_a][ck_b]

The checksum is the 8-bit Fletcher checksum computed over class, id, length
and payload. See u-blox interface description (UBX-13003221) for details.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Optional

SYNC1 = 0xB5
SYNC2 = 0x62

# Message identifiers we currently decode.
CLS_NAV = 0x01
ID_NAV_PVT = 0x07
ID_NAV_POSLLH = 0x02
ID_NAV_STATUS = 0x03

# Maximum payload length we will accept. The UBX length field is 16 bits
# (up to 65535), but legitimate u-blox messages stay well below 1 KB. A cap
# protects us from desynchronised streams that decode garbage as length.
MAX_PAYLOAD_LEN = 4096


@dataclass(frozen=True)
class UbxFrame:
    """A single decoded UBX frame (header + raw payload)."""

    msg_class: int
    msg_id: int
    payload: bytes

    @property
    def key(self) -> tuple[int, int]:
        return (self.msg_class, self.msg_id)


@dataclass(frozen=True)
class NavPVT:
    """NAV-PVT decoded fields (class 0x01, id 0x07, length 92)."""

    i_tow_ms: int
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: int
    valid: int
    t_acc_ns: int
    nano_ns: int
    fix_type: int
    flags: int
    flags2: int
    num_sv: int
    lon_deg: float
    lat_deg: float
    height_m: float
    h_msl_m: float
    h_acc_m: float
    v_acc_m: float
    vel_n_mps: float
    vel_e_mps: float
    vel_d_mps: float
    g_speed_mps: float
    head_mot_deg: float
    s_acc_mps: float
    head_acc_deg: float
    p_dop: float


def checksum(class_id: int, msg_id: int, payload: bytes) -> tuple[int, int]:
    """Compute UBX 8-bit Fletcher checksum over header + payload."""
    length = len(payload)
    ck_a = 0
    ck_b = 0
    for b in (class_id, msg_id, length & 0xFF, (length >> 8) & 0xFF):
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def iter_frames(data: bytes) -> Iterator[UbxFrame]:
    """Yield every well-formed UBX frame found in ``data``.

    Bytes that don't form a valid frame (bad sync, bad checksum, or truncated
    payload at end of buffer) are silently skipped — UBX streams are commonly
    interleaved with NMEA or partial captures.
    """
    n = len(data)
    i = 0
    while i + 8 <= n:
        if data[i] != SYNC1 or data[i + 1] != SYNC2:
            i += 1
            continue
        msg_class = data[i + 2]
        msg_id = data[i + 3]
        length = data[i + 4] | (data[i + 5] << 8)
        if length > MAX_PAYLOAD_LEN:
            i += 1
            continue
        end = i + 6 + length + 2
        if end > n:
            return
        payload = data[i + 6:i + 6 + length]
        ck_a, ck_b = checksum(msg_class, msg_id, payload)
        if data[end - 2] == ck_a and data[end - 1] == ck_b:
            yield UbxFrame(msg_class, msg_id, bytes(payload))
            i = end
        else:
            # Bad checksum — likely a false sync match. Skip past sync byte.
            i += 1


# Format covers fields up through pDOP. Real receivers send 92-byte NAV-PVT
# frames; the trailing 14 bytes (flags3, reserved, headVeh, magDec, magAcc)
# are not decoded here, so we only require the prefix this struct consumes.
_NAV_PVT_STRUCT = struct.Struct('<IHBBBBBBIiBBBBiiiiIIiiiiiIIH')


def decode_nav_pvt(payload: bytes) -> Optional[NavPVT]:
    """Decode a NAV-PVT payload. Returns ``None`` on length mismatch."""
    if len(payload) < _NAV_PVT_STRUCT.size:
        return None
    fields = _NAV_PVT_STRUCT.unpack_from(payload, 0)
    (i_tow, year, month, day, hour, minute, second, valid, t_acc, nano,
     fix_type, flags, flags2, num_sv, lon, lat, height, h_msl, h_acc, v_acc,
     vel_n, vel_e, vel_d, g_speed, head_mot, s_acc, head_acc, p_dop) = fields

    return NavPVT(
        i_tow_ms=i_tow,
        year=year,
        month=month,
        day=day,
        hour=hour,
        minute=minute,
        second=second,
        valid=valid,
        t_acc_ns=t_acc,
        nano_ns=nano,
        fix_type=fix_type,
        flags=flags,
        flags2=flags2,
        num_sv=num_sv,
        lon_deg=lon * 1e-7,
        lat_deg=lat * 1e-7,
        height_m=height * 1e-3,
        h_msl_m=h_msl * 1e-3,
        h_acc_m=h_acc * 1e-3,
        v_acc_m=v_acc * 1e-3,
        vel_n_mps=vel_n * 1e-3,
        vel_e_mps=vel_e * 1e-3,
        vel_d_mps=vel_d * 1e-3,
        g_speed_mps=g_speed * 1e-3,
        head_mot_deg=head_mot * 1e-5,
        s_acc_mps=s_acc * 1e-3,
        head_acc_deg=head_acc * 1e-5,
        p_dop=p_dop * 0.01,
    )


def encode_frame(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    """Encode a UBX frame (mostly used by tests)."""
    length = len(payload)
    header = bytes([SYNC1, SYNC2, msg_class, msg_id, length & 0xFF, (length >> 8) & 0xFF])
    ck_a, ck_b = checksum(msg_class, msg_id, payload)
    return header + payload + bytes([ck_a, ck_b])
