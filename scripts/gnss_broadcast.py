#!/usr/bin/env python3
"""Small, dependency-free RINEX 3 broadcast ephemeris utility.

The orbit models and constants follow RTKLIB's ``ephemeris.c`` implementation
for GPS/Galileo/BeiDou Keplerian ephemerides and GLONASS state-vector
ephemerides.  Times exposed by this module use a continuous GPST-like Unix
axis: UTC Unix time plus the configured GPS-UTC leap-second offset.
"""

import datetime as dt
import gzip
import math
import re
from collections import Counter, defaultdict
from dataclasses import dataclass


GPS_EPOCH_UNIX = 315964800.0
BDT_EPOCH_UNIX = 1136073600.0
WEEK_SECONDS = 604800.0
CLIGHT = 299792458.0

MU_GPS = 3.9860050e14
MU_GLO = 3.9860044e14
MU_GAL = 3.986004418e14
MU_CMP = 3.986004418e14

OMGE = 7.2921151467e-5
OMGE_GLO = 7.292115e-5
OMGE_GAL = 7.2921151467e-5
OMGE_CMP = 7.292115e-5

RE_GLO = 6378136.0
J2_GLO = 1.0826257e-3
GLO_STEP_SECONDS = 60.0

SIN_NEG_5 = -0.0871557427476582
COS_NEG_5 = 0.9961946980917456

MAX_EPHEMERIS_AGE_SECONDS = {
    "G": 7200.0,
    "E": 10800.0,
    "C": 21600.0,
    "R": 1800.0,
}

TIME_SCALE_TO_GPST_SECONDS = {
    "G": 0.0,
    "E": 0.0,
    "C": 14.0,
}


@dataclass
class KeplerEphemeris:
    sat_id: str
    sys: str
    prn: int
    toc_gpst: float
    toe_gpst: float
    toes: float
    week: int
    iode: int
    iodc: int
    code: int
    health: int
    f0: float
    f1: float
    f2: float
    sqrt_a: float
    eccentricity: float
    i0: float
    omega0: float
    omega: float
    m0: float
    delta_n: float
    omega_dot: float
    idot: float
    crc: float
    crs: float
    cuc: float
    cus: float
    cic: float
    cis: float


@dataclass
class GlonassEphemeris:
    sat_id: str
    sys: str
    prn: int
    toc_gpst: float
    toe_gpst: float
    health: int
    frequency_channel: int
    age_days: int
    tau_n: float
    gamma_n: float
    position_m: tuple
    velocity_mps: tuple
    acceleration_mps2: tuple


@dataclass
class NavigationAudit:
    version: float
    raw_records: int
    unique_records: int
    duplicate_records: int
    raw_by_system: dict
    unique_by_system: dict


def _open_text(path):
    if str(path).lower().endswith(".gz"):
        return gzip.open(path, "rt", encoding="ascii", errors="replace")
    return open(path, "r", encoding="ascii", errors="replace", newline=None)


def _rinex_float(field):
    text = field.strip().replace("D", "E").replace("d", "e")
    return float(text) if text else 0.0


def _parse_calendar(field):
    parts = field.split()
    if len(parts) != 6:
        raise ValueError("Invalid RINEX navigation epoch: %r" % field)
    year, month, day, hour, minute = map(int, parts[:5])
    second = float(parts[5])
    whole = int(math.floor(second))
    microsecond = int(round((second - whole) * 1e6))
    if microsecond == 1000000:
        whole += 1
        microsecond = 0
    return dt.datetime(
        year,
        month,
        day,
        hour,
        minute,
        whole,
        microsecond,
        tzinfo=dt.timezone.utc,
    ).timestamp()


def _absolute_toe(system, week, toes, toc_gpst):
    if system in ("G", "E"):
        toe = GPS_EPOCH_UNIX + week * WEEK_SECONDS + toes
    elif system == "C":
        toe = BDT_EPOCH_UNIX + week * WEEK_SECONDS + toes + 14.0
    else:
        raise ValueError("No Keplerian Toe conversion for system %s" % system)

    while toe - toc_gpst > WEEK_SECONDS / 2.0:
        toe -= WEEK_SECONDS
    while toe - toc_gpst < -WEEK_SECONDS / 2.0:
        toe += WEEK_SECONDS
    return toe


def _decode_kepler(sat_id, toc_calendar, data):
    system = sat_id[0]
    prn = int(sat_id[1:])
    if len(data) < 31:
        raise ValueError("Incomplete ephemeris record for %s" % sat_id)

    toc_gpst = toc_calendar + TIME_SCALE_TO_GPST_SECONDS[system]
    week = int(round(data[21]))
    toes = data[11]
    toe_gpst = _absolute_toe(system, week, toes, toc_gpst)

    iode = int(round(data[3]))
    iodc = 0
    code = 0
    if system == "G":
        iodc = int(round(data[26]))
        code = int(round(data[20]))
    elif system == "E":
        code = int(round(data[20]))
    elif system == "C":
        iodc = int(round(data[28]))

    return KeplerEphemeris(
        sat_id=sat_id,
        sys=system,
        prn=prn,
        toc_gpst=toc_gpst,
        toe_gpst=toe_gpst,
        toes=toes,
        week=week,
        iode=iode,
        iodc=iodc,
        code=code,
        health=int(round(data[24])),
        f0=data[0],
        f1=data[1],
        f2=data[2],
        sqrt_a=data[10],
        eccentricity=data[8],
        i0=data[15],
        omega0=data[13],
        omega=data[17],
        m0=data[6],
        delta_n=data[5],
        omega_dot=data[18],
        idot=data[19],
        crc=data[16],
        crs=data[4],
        cuc=data[7],
        cus=data[9],
        cic=data[12],
        cis=data[14],
    )


def _decode_glonass(sat_id, toc_calendar, data, leap_seconds):
    if len(data) < 15:
        raise ValueError("Incomplete GLONASS record for %s" % sat_id)

    # GLONASS navigation epochs are UTC. RTKLIB rounds Toc to the nearest
    # 15-minute boundary before converting it to GPST.
    toc_utc = math.floor((toc_calendar + 450.0) / 900.0) * 900.0
    toe_gpst = toc_utc + leap_seconds
    frequency_channel = int(round(data[10]))
    if frequency_channel > 128:
        frequency_channel -= 256

    return GlonassEphemeris(
        sat_id=sat_id,
        sys="R",
        prn=int(sat_id[1:]),
        toc_gpst=toe_gpst,
        toe_gpst=toe_gpst,
        health=int(round(data[6])),
        frequency_channel=frequency_channel,
        age_days=int(round(data[14])),
        tau_n=-data[0],
        gamma_n=data[1],
        position_m=(data[3] * 1e3, data[7] * 1e3, data[11] * 1e3),
        velocity_mps=(data[4] * 1e3, data[8] * 1e3, data[12] * 1e3),
        acceleration_mps2=(data[5] * 1e3, data[9] * 1e3, data[13] * 1e3),
    )


def _deduplicate(records):
    selected = {}
    for record in records:
        if record.sys == "R":
            key = (record.sat_id, round(record.toe_gpst, 3))
        else:
            key = (
                record.sat_id,
                round(record.toe_gpst, 3),
                record.iode,
                record.code,
            )
        old = selected.get(key)
        if old is None:
            selected[key] = record
            continue
        old_rank = (old.health != 0, -old.toc_gpst)
        new_rank = (record.health != 0, -record.toc_gpst)
        if new_rank < old_rank:
            selected[key] = record
    return list(selected.values())


def read_rinex3_navigation(path, leap_seconds=18.0):
    """Read G/R/E/C records from a mixed RINEX 3 navigation file."""
    raw_records = []
    raw_by_system = Counter()

    with _open_text(path) as handle:
        first = handle.readline()
        if not first:
            raise ValueError("Empty RINEX navigation file")
        try:
            version = float(first[:9])
        except ValueError as exc:
            raise ValueError("Invalid RINEX version line") from exc
        if not (3.0 <= version < 4.0):
            raise ValueError(
                "This script supports RINEX 3 navigation; found %.2f" % version
            )

        for line in handle:
            if line[60:80].strip() == "END OF HEADER":
                break
        else:
            raise ValueError("RINEX END OF HEADER not found")

        while True:
            line = handle.readline()
            if not line:
                break
            if not re.match(r"^[GREC][0-9]{2}", line):
                continue

            sat_id = line[:3]
            system = sat_id[0]
            continuation_count = 3 if system == "R" else 7
            toc_calendar = _parse_calendar(line[4:23])
            data = [_rinex_float(line[23 + 19 * i : 42 + 19 * i]) for i in range(3)]

            for _ in range(continuation_count):
                continuation = handle.readline()
                if not continuation:
                    raise ValueError("Unexpected end of RINEX record for %s" % sat_id)
                data.extend(
                    _rinex_float(continuation[4 + 19 * i : 23 + 19 * i])
                    for i in range(4)
                )

            if system == "R":
                record = _decode_glonass(
                    sat_id, toc_calendar, data, leap_seconds
                )
            else:
                record = _decode_kepler(sat_id, toc_calendar, data)
            raw_records.append(record)
            raw_by_system[system] += 1

    unique_records = _deduplicate(raw_records)
    grouped = defaultdict(list)
    unique_by_system = Counter()
    for record in unique_records:
        grouped[record.sat_id].append(record)
        unique_by_system[record.sys] += 1
    for records in grouped.values():
        records.sort(key=lambda item: item.toe_gpst)

    audit = NavigationAudit(
        version=version,
        raw_records=len(raw_records),
        unique_records=len(unique_records),
        duplicate_records=len(raw_records) - len(unique_records),
        raw_by_system=dict(sorted(raw_by_system.items())),
        unique_by_system=dict(sorted(unique_by_system.items())),
    )
    return dict(grouped), audit


def select_ephemeris(records, receive_gpst):
    if not records:
        return None, None, None
    system = records[0].sys
    limit = MAX_EPHEMERIS_AGE_SECONDS[system]
    candidates = []
    for record in records:
        age = receive_gpst - record.toe_gpst
        if abs(age) <= limit:
            candidates.append((abs(age), record.health != 0, -record.toc_gpst, record, age))
    if not candidates:
        nearest = min(records, key=lambda item: abs(receive_gpst - item.toe_gpst))
        return None, receive_gpst - nearest.toe_gpst, limit
    _, _, _, record, age = min(candidates, key=lambda item: item[:3])
    return record, age, limit


def _kepler_position(record, gpst):
    if record.sqrt_a <= 0.0:
        raise ValueError("Invalid semi-major axis for %s" % record.sat_id)
    if record.sys == "G":
        mu, earth_rate = MU_GPS, OMGE
    elif record.sys == "E":
        mu, earth_rate = MU_GAL, OMGE_GAL
    elif record.sys == "C":
        mu, earth_rate = MU_CMP, OMGE_CMP
    else:
        raise ValueError("Unsupported Keplerian system %s" % record.sys)

    semi_major = record.sqrt_a * record.sqrt_a
    tk = gpst - record.toe_gpst
    mean_anomaly = record.m0 + (
        math.sqrt(mu / (semi_major ** 3)) + record.delta_n
    ) * tk

    eccentric_anomaly = mean_anomaly
    for _ in range(30):
        previous = eccentric_anomaly
        eccentric_anomaly -= (
            eccentric_anomaly
            - record.eccentricity * math.sin(eccentric_anomaly)
            - mean_anomaly
        ) / (1.0 - record.eccentricity * math.cos(eccentric_anomaly))
        if abs(eccentric_anomaly - previous) <= 1e-14:
            break
    else:
        raise RuntimeError("Kepler iteration failed for %s" % record.sat_id)

    sin_e = math.sin(eccentric_anomaly)
    cos_e = math.cos(eccentric_anomaly)
    argument = math.atan2(
        math.sqrt(1.0 - record.eccentricity ** 2) * sin_e,
        cos_e - record.eccentricity,
    ) + record.omega
    radius = semi_major * (1.0 - record.eccentricity * cos_e)
    inclination = record.i0 + record.idot * tk
    sin_2u = math.sin(2.0 * argument)
    cos_2u = math.cos(2.0 * argument)
    argument += record.cus * sin_2u + record.cuc * cos_2u
    radius += record.crs * sin_2u + record.crc * cos_2u
    inclination += record.cis * sin_2u + record.cic * cos_2u

    x_orbit = radius * math.cos(argument)
    y_orbit = radius * math.sin(argument)
    cos_i = math.cos(inclination)

    if record.sys == "C" and record.prn <= 5:
        omega = record.omega0 + record.omega_dot * tk - earth_rate * record.toes
        sin_o, cos_o = math.sin(omega), math.cos(omega)
        xg = x_orbit * cos_o - y_orbit * cos_i * sin_o
        yg = x_orbit * sin_o + y_orbit * cos_i * cos_o
        zg = y_orbit * math.sin(inclination)
        sin_rotation = math.sin(earth_rate * tk)
        cos_rotation = math.cos(earth_rate * tk)
        return (
            xg * cos_rotation
            + yg * sin_rotation * COS_NEG_5
            + zg * sin_rotation * SIN_NEG_5,
            -xg * sin_rotation
            + yg * cos_rotation * COS_NEG_5
            + zg * cos_rotation * SIN_NEG_5,
            -yg * SIN_NEG_5 + zg * COS_NEG_5,
        )

    omega = (
        record.omega0
        + (record.omega_dot - earth_rate) * tk
        - earth_rate * record.toes
    )
    sin_o, cos_o = math.sin(omega), math.cos(omega)
    return (
        x_orbit * cos_o - y_orbit * cos_i * sin_o,
        x_orbit * sin_o + y_orbit * cos_i * cos_o,
        y_orbit * math.sin(inclination),
    )


def _glonass_derivative(state, acceleration):
    x, y, z, vx, vy, vz = state
    radius_squared = x * x + y * y + z * z
    if radius_squared <= 0.0:
        raise ValueError("Invalid GLONASS state radius")
    radius_cubed = radius_squared * math.sqrt(radius_squared)
    a = (
        1.5
        * J2_GLO
        * MU_GLO
        * RE_GLO ** 2
        / radius_squared
        / radius_cubed
    )
    b = 5.0 * z * z / radius_squared
    c = -MU_GLO / radius_cubed - a * (1.0 - b)
    omega_squared = OMGE_GLO ** 2
    return (
        vx,
        vy,
        vz,
        (c + omega_squared) * x + 2.0 * OMGE_GLO * vy + acceleration[0],
        (c + omega_squared) * y - 2.0 * OMGE_GLO * vx + acceleration[1],
        (c - 2.0 * a) * z + acceleration[2],
    )


def _rk4_step(state, step, acceleration):
    k1 = _glonass_derivative(state, acceleration)
    w = tuple(state[i] + k1[i] * step / 2.0 for i in range(6))
    k2 = _glonass_derivative(w, acceleration)
    w = tuple(state[i] + k2[i] * step / 2.0 for i in range(6))
    k3 = _glonass_derivative(w, acceleration)
    w = tuple(state[i] + k3[i] * step for i in range(6))
    k4 = _glonass_derivative(w, acceleration)
    return tuple(
        state[i] + (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * step / 6.0
        for i in range(6)
    )


def _glonass_position(record, gpst):
    remaining = gpst - record.toe_gpst
    state = tuple(record.position_m) + tuple(record.velocity_mps)
    while abs(remaining) > 1e-9:
        step = math.copysign(min(abs(remaining), GLO_STEP_SECONDS), remaining)
        state = _rk4_step(state, step, record.acceleration_mps2)
        remaining -= step
    return state[:3]


def satellite_position(record, gpst):
    if record.sys == "R":
        return _glonass_position(record, gpst)
    return _kepler_position(record, gpst)


def satellite_clock(record, gpst):
    if record.sys == "R":
        delta = gpst - record.toe_gpst
        return -record.tau_n + record.gamma_n * delta
    delta = gpst - record.toc_gpst
    clock = record.f0 + record.f1 * delta + record.f2 * delta * delta
    if record.sys == "G":
        mu = MU_GPS
    elif record.sys == "E":
        mu = MU_GAL
    else:
        mu = MU_CMP
    semi_major = record.sqrt_a * record.sqrt_a
    tk = gpst - record.toe_gpst
    mean_anomaly = record.m0 + (
        math.sqrt(mu / (semi_major ** 3)) + record.delta_n
    ) * tk
    eccentric_anomaly = mean_anomaly
    for _ in range(30):
        previous = eccentric_anomaly
        eccentric_anomaly -= (
            eccentric_anomaly
            - record.eccentricity * math.sin(eccentric_anomaly)
            - mean_anomaly
        ) / (1.0 - record.eccentricity * math.cos(eccentric_anomaly))
        if abs(eccentric_anomaly - previous) <= 1e-14:
            break
    clock -= (
        2.0
        * math.sqrt(mu * semi_major)
        * record.eccentricity
        * math.sin(eccentric_anomaly)
        / (CLIGHT ** 2)
    )
    return clock


def rotate_transmit_ecef_to_receive_frame(position, travel_time):
    angle = OMGE * travel_time
    cosine, sine = math.cos(angle), math.sin(angle)
    x, y, z = position
    return (cosine * x + sine * y, -sine * x + cosine * y, z)


def norm3(vector):
    return math.sqrt(sum(component * component for component in vector))


def subtract3(left, right):
    return tuple(left[index] - right[index] for index in range(3))


def transmit_geometry(record, receive_gpst, receiver_ecef, iterations=3):
    travel_time = 0.075
    rotated = None
    transmit_gpst = receive_gpst - travel_time
    for _ in range(iterations):
        transmit_gpst = receive_gpst - travel_time
        position = satellite_position(record, transmit_gpst)
        rotated = rotate_transmit_ecef_to_receive_frame(position, travel_time)
        geometric_range = norm3(subtract3(rotated, receiver_ecef))
        travel_time = geometric_range / CLIGHT
    transmit_gpst = receive_gpst - travel_time
    position = satellite_position(record, transmit_gpst)
    rotated = rotate_transmit_ecef_to_receive_frame(position, travel_time)
    geometric_range = norm3(subtract3(rotated, receiver_ecef))
    return rotated, geometric_range, travel_time, transmit_gpst


def ecef_to_geodetic(ecef):
    x, y, z = ecef
    semi_major = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_squared = flattening * (2.0 - flattening)
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    latitude = math.atan2(z, horizontal * (1.0 - eccentricity_squared))
    height = 0.0
    for _ in range(10):
        sine = math.sin(latitude)
        prime_vertical = semi_major / math.sqrt(
            1.0 - eccentricity_squared * sine * sine
        )
        height = horizontal / max(math.cos(latitude), 1e-15) - prime_vertical
        new_latitude = math.atan2(
            z,
            horizontal
            * (1.0 - eccentricity_squared * prime_vertical / (prime_vertical + height)),
        )
        if abs(new_latitude - latitude) < 1e-13:
            latitude = new_latitude
            break
        latitude = new_latitude
    return math.degrees(latitude), math.degrees(longitude), height


def geodetic_to_ecef(latitude_deg, longitude_deg, height):
    semi_major = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_squared = flattening * (2.0 - flattening)
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sine = math.sin(latitude)
    cosine = math.cos(latitude)
    prime_vertical = semi_major / math.sqrt(
        1.0 - eccentricity_squared * sine * sine
    )
    return (
        (prime_vertical + height) * cosine * math.cos(longitude),
        (prime_vertical + height) * cosine * math.sin(longitude),
        (prime_vertical * (1.0 - eccentricity_squared) + height) * sine,
    )


def elevation_azimuth(receiver_ecef, satellite_ecef):
    latitude_deg, longitude_deg, _ = ecef_to_geodetic(receiver_ecef)
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    dx, dy, dz = subtract3(satellite_ecef, receiver_ecef)
    east = -math.sin(longitude) * dx + math.cos(longitude) * dy
    north = (
        -math.sin(latitude) * math.cos(longitude) * dx
        - math.sin(latitude) * math.sin(longitude) * dy
        + math.cos(latitude) * dz
    )
    up = (
        math.cos(latitude) * math.cos(longitude) * dx
        + math.cos(latitude) * math.sin(longitude) * dy
        + math.sin(latitude) * dz
    )
    azimuth = math.degrees(math.atan2(east, north)) % 360.0
    elevation = math.degrees(math.atan2(up, math.hypot(east, north)))
    return elevation, azimuth
