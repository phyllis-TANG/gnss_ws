#!/usr/bin/env python3
"""
ubx_to_rosbag.py
Convert UBX file → ROS1 bag for del1RTK SPP

Topics written:
  /ublox_driver/range_meas   gnss_comm/GnssMeasMsg
  /ublox_driver/ephem        gnss_comm/GnssEphemMsg
  /ublox_driver/receiver_lla sensor_msgs/NavSatFix
"""

import sys, math, struct, warnings, numpy as np
from pathlib import Path
from datetime import datetime, timezone

warnings.filterwarnings('ignore')

# ── deps ─────────────────────────────────────────────────────────────────────
try:
    from pyubx2 import UBXReader
except ImportError:
    sys.exit("pip install pyubx2")
try:
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    sys.exit("pip install rosbags")

# ── satellite numbering (mirrors gnss_comm/gnss_utility.hpp) ─────────────────
N_GPS, N_GLO, N_GAL = 32, 27, 38

def sat_no(gnss_id, prn):
    """Return gnss_comm internal satellite number (1-based)."""
    if gnss_id == 0:   return prn                              # GPS
    if gnss_id == 6:   return N_GPS + prn                     # GLO
    if gnss_id == 2:   return N_GPS + N_GLO + prn             # GAL
    if gnss_id == 3:   return N_GPS + N_GLO + N_GAL + prn     # BDS
    return 0  # unsupported (SBAS, QZSS, NavIC …)

# ── signal frequency table (Hz) ───────────────────────────────────────────────
SIG_FREQ = {
    0: {0: 1575.42e6, 1: 1575.42e6, 3: 1227.60e6, 4: 1227.60e6,
        6: 1176.45e6, 7: 1176.45e6},          # GPS
    2: {0: 1575.42e6, 1: 1575.42e6, 3: 1176.45e6, 4: 1176.45e6,
        5: 1207.14e6, 6: 1207.14e6},          # Galileo
    3: {0: 1561.098e6, 1: 1561.098e6, 2: 1207.14e6, 3: 1207.14e6,
        5: 1176.45e6,  6: 1176.45e6},         # BeiDou
}

# ── prStd index → std in metres (UBX spec: 0.01 * 2^n) ──────────────────────
def prstd_m(idx):
    return 0.01 * (2 ** idx) if idx > 0 else 1.0

# ── GPS week / ToW helpers ────────────────────────────────────────────────────
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)

def gps_tow_to_unix(week, tow):
    return (GPS_EPOCH.timestamp()) + week * 604800 + tow

# ── RINEX 3 NAV parser (GPS only) ────────────────────────────────────────────
def parse_rinex_nav(path):
    """Return list of dicts, one per GPS satellite ephemeris entry."""
    ephems = []
    lines = Path(path).read_text().splitlines()
    i = 0
    # skip header
    while i < len(lines):
        if 'END OF HEADER' in lines[i]:
            i += 1; break
        i += 1

    def val(s):
        return float(s.replace('D', 'E').replace('d', 'e'))

    while i < len(lines):
        line = lines[i]
        if not line or line[0] not in 'GECJRS':
            i += 1; continue
        sys_char = line[0]
        # SBAS (S) and GLONASS (R) have 4-line entries; GPS/GAL/BDS/QZSS have 8
        if sys_char in ('S', 'R'):
            i += 4; continue
        if sys_char != 'G':          # GPS only
            i += 8; continue
        prn = int(line[1:3])
        # toc: year month day hour min sec
        yr   = int(line[4:8])
        mo   = int(line[9:11])
        dy   = int(line[12:14])
        hr   = int(line[15:17])
        mn   = int(line[18:20])
        sec  = float(line[21:23])
        af0  = val(line[23:42])
        af1  = val(line[42:61])
        af2  = val(line[61:80])

        def row(offset):
            return lines[i + offset][4:] if i + offset < len(lines) else ''

        def fields(r):
            return [val(r[k:k+19]) for k in range(0, len(r.rstrip()), 19) if r[k:k+19].strip()]

        try:
            f2 = fields(row(1))  # IODE Crs delta_n M0
            f3 = fields(row(2))  # Cuc e Cus sqrt_A
            f4 = fields(row(3))  # toe Cic OMG0 Cis
            f5 = fields(row(4))  # i0 Crc omega OMG_dot
            f6 = fields(row(5))  # i_dot L2codes week L2flag
            f7 = fields(row(6))  # ura health tgd iodc
            f8 = fields(row(7))  # ttr fit
        except Exception:
            i += 8; continue

        # GPS week from nav (may be < 1024)
        nav_week = int(f6[2]) if len(f6) > 2 else 0
        # adjust for week rollover: if nav_week < 2000 assume it's 2024+ era
        if nav_week < 1000:
            nav_week += 2048

        sqrt_A = f3[3]
        A = sqrt_A ** 2

        # toc as GPS time
        # approximate: convert calendar to GPS week/tow
        from datetime import datetime, timezone
        dt = datetime(yr, mo, dy, hr, mn, int(sec), tzinfo=timezone.utc)
        gps_sec = (dt - GPS_EPOCH).total_seconds()
        toc_week = int(gps_sec // 604800)
        toc_tow  = gps_sec % 604800

        toe_tow = f4[0]
        toe_week = toc_week  # same week as toc

        ephems.append({
            'prn':     prn,
            'toc_week': toc_week, 'toc_tow': toc_tow,
            'toe_week': nav_week,  'toe_tow': toe_tow,
            'af0': af0, 'af1': af1, 'af2': af2,
            'iode': int(f2[0]), 'iodc': int(f7[3]) if len(f7) > 3 else 0,
            'crs': f2[1], 'delta_n': f2[2], 'M0': f2[3],
            'cuc': f3[0], 'e': f3[1], 'cus': f3[2],
            'sqrt_A': sqrt_A, 'A': A,
            'toe_tow_field': toe_tow,
            'cic': f4[1], 'OMG0': f4[2], 'cis': f4[3],
            'i0': f5[0], 'crc': f5[1], 'omg': f5[2], 'OMG_dot': f5[3],
            'i_dot': f6[0],
            'ura': f7[0], 'health': int(f7[1]), 'tgd': f7[2],
            'week': nav_week,
            'code': 1,  # L2 codes (GPS)
        })
        i += 8

    print(f"[NAV] Parsed {len(ephems)} GPS ephemerides")
    return ephems

# ── register gnss_comm message types ─────────────────────────────────────────
def make_typestore():
    ts = get_typestore(Stores.ROS1_NOETIC)

    DEFS = {
        'gnss_comm/msg/GnssTimeMsg': "uint32 week\nfloat64 tow\n",
        'gnss_comm/msg/GnssObsMsg':  """\
gnss_comm/GnssTimeMsg time
uint32 sat
float64[] freqs
float64[] CN0
uint8[] LLI
uint8[] code
float64[] psr
float64[] psr_std
float64[] cp
float64[] cp_std
float64[] dopp
float64[] dopp_std
uint8[] status
""",
        'gnss_comm/msg/GnssMeasMsg': "gnss_comm/GnssObsMsg[] meas\n",
        'gnss_comm/msg/GnssEphemMsg': """\
uint32 sat
gnss_comm/GnssTimeMsg ttr
gnss_comm/GnssTimeMsg toe
gnss_comm/GnssTimeMsg toc
float64 toe_tow
uint32 week
uint32 iode
uint32 iodc
uint32 health
uint32 code
float64 ura
float64 A
float64 e
float64 i0
float64 omg
float64 OMG0
float64 M0
float64 delta_n
float64 OMG_dot
float64 i_dot
float64 cuc
float64 cus
float64 crc
float64 crs
float64 cic
float64 cis
float64 af0
float64 af1
float64 af2
float64 tgd0
float64 tgd1
float64 A_dot
float64 n_dot
""",
    }

    add_types = {}
    for name, defn in DEFS.items():
        add_types.update(get_types_from_msg(defn, name))
    ts.register(add_types)
    return ts

# ── main conversion ───────────────────────────────────────────────────────────
def convert(ubx_path, nav_path, bag_path):
    ts   = make_typestore()
    GnssMeasMsg  = ts.types['gnss_comm/msg/GnssMeasMsg']
    GnssObsMsg   = ts.types['gnss_comm/msg/GnssObsMsg']
    GnssTimeMsg  = ts.types['gnss_comm/msg/GnssTimeMsg']
    GnssEphemMsg = ts.types['gnss_comm/msg/GnssEphemMsg']
    NavSatFix    = ts.types['sensor_msgs/msg/NavSatFix']
    NavSatStatus = ts.types['sensor_msgs/msg/NavSatStatus']
    Header       = ts.types['std_msgs/msg/Header']
    Time         = ts.types['builtin_interfaces/msg/Time']

    ephems = parse_rinex_nav(nav_path)

    n_meas = 0; n_pvt = 0; n_eph = 0

    with Writer(bag_path) as writer:
        # register topics
        conn_meas = writer.add_connection('/ublox_driver/range_meas',
                                          'gnss_comm/msg/GnssMeasMsg',
                                          typestore=ts)
        conn_eph  = writer.add_connection('/ublox_driver/ephem',
                                          'gnss_comm/msg/GnssEphemMsg',
                                          typestore=ts)
        conn_lla  = writer.add_connection('/ublox_driver/receiver_lla',
                                          'sensor_msgs/msg/NavSatFix',
                                          typestore=ts)

        with open(ubx_path, 'rb') as f:
            ubr = UBXReader(f, protfilter=2, validate=0, msgmode=0)
            first_stamp = None

            for _, parsed in ubr:
                if parsed is None:
                    continue

                # ── RXM-RAWX → GnssMeasMsg ──────────────────────────────────
                if parsed.identity == 'RXM-RAWX':
                    week = parsed.week
                    tow  = parsed.rcvTow
                    unix_ns = int(gps_tow_to_unix(week, tow) * 1e9)

                    if first_stamp is None:
                        first_stamp = unix_ns
                        # write ephemerides before first measurement
                        for eph in ephems:
                            prn   = eph['prn']
                            s_no  = sat_no(0, prn)
                            if s_no == 0:
                                continue
                            ttr   = GnssTimeMsg(week=np.uint32(week),
                                                tow=tow)
                            toe   = GnssTimeMsg(week=np.uint32(eph['toe_week']),
                                                tow=float(eph['toe_tow']))
                            toc   = GnssTimeMsg(week=np.uint32(eph['toc_week']),
                                                tow=float(eph['toc_tow']))
                            emsg  = GnssEphemMsg(
                                sat=np.uint32(s_no),
                                ttr=ttr, toe=toe, toc=toc,
                                toe_tow=float(eph['toe_tow_field']),
                                week=np.uint32(eph['week']),
                                iode=np.uint32(eph['iode']),
                                iodc=np.uint32(eph['iodc']),
                                health=np.uint32(eph['health']),
                                code=np.uint32(eph['code']),
                                ura=float(eph['ura']),
                                A=float(eph['A']),
                                e=float(eph['e']),
                                i0=float(eph['i0']),
                                omg=float(eph['omg']),
                                OMG0=float(eph['OMG0']),
                                M0=float(eph['M0']),
                                delta_n=float(eph['delta_n']),
                                OMG_dot=float(eph['OMG_dot']),
                                i_dot=float(eph['i_dot']),
                                cuc=float(eph['cuc']),
                                cus=float(eph['cus']),
                                crc=float(eph['crc']),
                                crs=float(eph['crs']),
                                cic=float(eph['cic']),
                                cis=float(eph['cis']),
                                af0=float(eph['af0']),
                                af1=float(eph['af1']),
                                af2=float(eph['af2']),
                                tgd0=float(eph['tgd']),
                                tgd1=0.0,
                                A_dot=0.0,
                                n_dot=0.0,
                            )
                            data = ts.serialize_ros1(emsg, 'gnss_comm/msg/GnssEphemMsg')
                            writer.write(conn_eph, unix_ns - 1_000_000, data)
                            n_eph += 1

                    obs_list = []
                    for idx in range(1, parsed.numMeas + 1):
                        gnss_id = getattr(parsed, f'gnssId_{idx:02d}')
                        sv_id   = getattr(parsed, f'svId_{idx:02d}')
                        sig_id  = getattr(parsed, f'sigId_{idx:02d}')
                        pr      = getattr(parsed, f'prMes_{idx:02d}')
                        cp      = getattr(parsed, f'cpMes_{idx:02d}')
                        dopp    = getattr(parsed, f'doMes_{idx:02d}')
                        cno     = getattr(parsed, f'cno_{idx:02d}')
                        pr_std_i= getattr(parsed, f'prStd_{idx:02d}')
                        cp_std_i= getattr(parsed, f'cpStd_{idx:02d}')
                        do_std_i= getattr(parsed, f'doStd_{idx:02d}')
                        pr_valid= getattr(parsed, f'prValid_{idx:02d}', 1)
                        cp_valid= getattr(parsed, f'cpValid_{idx:02d}', 0)
                        half_cyc= getattr(parsed, f'halfCyc_{idx:02d}', 0)
                        sub_half= getattr(parsed, f'subHalfCyc_{idx:02d}', 0)

                        s_no = sat_no(gnss_id, sv_id)
                        if s_no == 0:
                            continue  # skip SBAS/QZSS/NavIC
                        if not pr_valid or pr <= 0:
                            continue

                        freq = SIG_FREQ.get(gnss_id, {}).get(sig_id, 0.0)
                        if freq == 0.0:
                            continue  # unknown signal

                        t_obs = GnssTimeMsg(week=np.uint32(week), tow=tow)
                        status_byte = np.uint8(
                            (1 if pr_valid else 0) |
                            (2 if cp_valid else 0) |
                            (4 if half_cyc else 0) |
                            (8 if sub_half else 0)
                        )

                        obs = GnssObsMsg(
                            time=t_obs,
                            sat=np.uint32(s_no),
                            freqs=np.array([freq], dtype=np.float64),
                            CN0=np.array([float(cno)], dtype=np.float64),
                            LLI=np.array([0], dtype=np.uint8),
                            code=np.array([1], dtype=np.uint8),  # CODE_L1C
                            psr=np.array([float(pr)], dtype=np.float64),
                            psr_std=np.array([prstd_m(pr_std_i)], dtype=np.float64),
                            cp=np.array([float(cp)], dtype=np.float64),
                            cp_std=np.array([0.004 * cp_std_i], dtype=np.float64),
                            dopp=np.array([float(dopp)], dtype=np.float64),
                            dopp_std=np.array([0.002 * (2 ** do_std_i)], dtype=np.float64),
                            status=np.array([status_byte], dtype=np.uint8),
                        )
                        obs_list.append(obs)

                    if obs_list:
                        meas_msg = GnssMeasMsg(meas=np.array(obs_list, dtype=object))
                        data = ts.serialize_ros1(meas_msg, 'gnss_comm/msg/GnssMeasMsg')
                        writer.write(conn_meas, unix_ns, data)
                        n_meas += 1

                # ── NAV-PVT → NavSatFix ─────────────────────────────────────
                elif parsed.identity == 'NAV-PVT':
                    if parsed.fixType < 2:
                        continue
                    itow_s = parsed.iTOW / 1000.0  # ms → s
                    # use iTOW to get GPS week (approximate from same week as RAWX)
                    # We'll use the wall clock from the first RAWX epoch as reference
                    if first_stamp is None:
                        continue
                    # approximate unix_ns from calendar time in NAV-PVT
                    dt = datetime(parsed.year, parsed.month, parsed.day,
                                  parsed.hour, parsed.min, parsed.second,
                                  tzinfo=timezone.utc)
                    unix_ns = int(dt.timestamp() * 1e9) + parsed.nano

                    hdr = Header(
                        seq=np.uint32(n_pvt),
                        stamp=Time(sec=int(unix_ns // 1_000_000_000),
                                   nanosec=int(unix_ns % 1_000_000_000)),
                        frame_id='',
                    )
                    status = NavSatStatus(
                        status=np.int8(0),   # STATUS_FIX
                        service=np.uint16(1) # SERVICE_GPS
                    )
                    fix = NavSatFix(
                        header=hdr,
                        status=status,
                        latitude=float(parsed.lat),           # pyubx2 already in degrees
                        longitude=float(parsed.lon),          # pyubx2 already in degrees
                        altitude=float(parsed.height) * 1e-3, # mm → m
                        position_covariance=np.zeros(9, dtype=np.float64),
                        position_covariance_type=np.uint8(0),
                    )
                    data = ts.serialize_ros1(fix, 'sensor_msgs/msg/NavSatFix')
                    writer.write(conn_lla, unix_ns, data)
                    n_pvt += 1

    print(f"[BAG] Written: {n_eph} ephemerides, {n_meas} meas epochs, {n_pvt} PVT fixes")
    print(f"[BAG] Output: {bag_path}")

# ── entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    UBX  = '/home/user/gnss_ws/2026-4-15_174245_serial-COM5(1).ubx'
    NAV  = '/home/user/gnss_ws/spp_results/nav.nav'
    BAG  = '/home/user/gnss_ws/spp_results/gnss_data.bag'
    convert(UBX, NAV, BAG)
