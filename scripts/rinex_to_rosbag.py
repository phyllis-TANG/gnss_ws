#!/usr/bin/env python3
"""
rinex_to_rosbag.py
RINEX 3.02 obs + nav → gnss_comm ROS1 bag，供 del1RTK SPP 使用
无额外依赖，只需 numpy（ROS 环境自带）

用法：
  python3 rinex_to_rosbag.py --obs <obs文件> --nav <nav文件> --out gnss_urbannav.bag
"""

import argparse, sys, numpy as np
from pathlib import Path
from datetime import datetime, timezone

# ── 卫星编号 ──────────────────────────────────────────────────────────────────
N_GPS, N_GLO, N_GAL = 32, 27, 38

def sat_no(sys_char, prn):
    if sys_char == 'G': return prn
    if sys_char == 'R': return N_GPS + prn
    if sys_char == 'E': return N_GPS + N_GLO + prn
    if sys_char == 'C': return N_GPS + N_GLO + N_GAL + prn
    return 0

# ── 时间转换 ──────────────────────────────────────────────────────────────────
GPS_EPOCH_UNIX = 315964800   # 1980-01-06 00:00:00 UTC
LEAP_SECONDS   = 18          # 2021 年有效

def epoch_to_unix(year, month, day, hour, minute, sec):
    dt = datetime(year, month, day, hour, minute, int(sec),
                  int((sec % 1) * 1e6), tzinfo=timezone.utc)
    return dt.timestamp()

def unix_to_gps(unix_t):
    """Unix 时间 → (GPS week, GPS tow)"""
    gps_secs = unix_t - GPS_EPOCH_UNIX + LEAP_SECONDS
    week = int(gps_secs // 604800)
    tow  = float(gps_secs % 604800)
    return week, tow

# ── L1 频率 ───────────────────────────────────────────────────────────────────
def l1_freq(sys_char, freqo=0):
    if sys_char == 'G': return 1575.42e6
    if sys_char == 'R': return (1602.0 + freqo * 0.5625) * 1e6
    if sys_char == 'E': return 1575.42e6
    if sys_char == 'C': return 1561.098e6
    return 1575.42e6

# ── RINEX 3 obs 解析 ───────────────────────────────────────────────────────────
def parse_obs(filepath):
    lines = Path(filepath).read_text(errors='ignore').splitlines()
    sys_obs_types = {}
    in_header = True
    i = 0
    while i < len(lines) and in_header:
        line = lines[i]
        label = line[60:].strip() if len(line) > 60 else ''
        if 'SYS / # / OBS TYPES' in label:
            sys_char = line[0]
            n = int(line[3:6])
            types = line[7:60].split()
            while len(types) < n:
                i += 1
                types += lines[i][7:60].split()
            sys_obs_types[sys_char] = types
        elif 'END OF HEADER' in label:
            in_header = False
        i += 1

    epochs = []
    while i < len(lines):
        line = lines[i]
        if not line.startswith('>'):
            i += 1
            continue
        try:
            year   = int(line[2:6]);  month  = int(line[7:9])
            day    = int(line[10:12]); hour   = int(line[13:15])
            minute = int(line[16:18]); sec    = float(line[19:29])
            n_sv   = int(line[32:35])
        except Exception:
            i += 1; continue

        t = epoch_to_unix(year, month, day, hour, minute, sec)
        sv_data = {}
        for _ in range(n_sv):
            i += 1
            if i >= len(lines): break
            sv_line = lines[i]
            if len(sv_line) < 3: continue
            sv = sv_line[:3]
            sys_char = sv[0]
            if sys_char not in sys_obs_types: continue
            obs_vals = {}
            for k, ot in enumerate(sys_obs_types[sys_char]):
                start = 3 + k * 16
                raw = sv_line[start:start+14].strip() if len(sv_line) > start else ''
                try:    obs_vals[ot] = float(raw)
                except: obs_vals[ot] = float('nan')
            sv_data[sv] = obs_vals
        epochs.append((t, sv_data))
        i += 1
    return epochs

# ── RINEX 3/2 nav 解析 ────────────────────────────────────────────────────────
def parse_nav(filepath):
    lines = Path(filepath).read_text(errors='ignore').splitlines()
    version = 3
    for line in lines[:20]:
        if 'RINEX VERSION' in (line[60:] if len(line) > 60 else ''):
            try: version = int(float(line[:9]))
            except: pass
        if 'END OF HEADER' in (line[60:] if len(line) > 60 else ''): break

    i = 0
    while i < len(lines):
        if 'END OF HEADER' in (lines[i][60:] if len(lines[i]) > 60 else ''):
            i += 1; break
        i += 1

    def rv(line, col):
        s = line[col:col+19].strip().replace('D','e').replace('d','e')
        try: return float(s)
        except: return 0.0

    ephems = []
    while i < len(lines):
        line = lines[i]
        if not line or line[0] == ' ': i += 1; continue
        try:
            if version >= 3:
                sys_char = line[0]; prn = int(line[1:3])
                year=int(line[4:8]); month=int(line[9:11]); day=int(line[12:14])
                hour=int(line[15:17]); minute=int(line[18:20]); sec=float(line[21:23])
                af0=rv(line,23); af1=rv(line,42); af2=rv(line,61)
            else:
                sys_char='G'; prn=int(line[0:2])
                year=int(line[3:5])+2000; month=int(line[6:8]); day=int(line[9:11])
                hour=int(line[12:14]); minute=int(line[15:17]); sec=float(line[17:22])
                af0=rv(line,22); af1=rv(line,41); af2=rv(line,60)
        except Exception: i += 1; continue

        if sys_char not in ('G','E','C','R'): i += 8; continue

        toc_unix = epoch_to_unix(year, month, day, hour, minute, sec)

        brd = []
        n_lines = 3 if sys_char == 'R' else 7
        for _ in range(n_lines):
            i += 1
            if i >= len(lines): break
            l = lines[i]
            for col in (4,23,42,61): brd.append(rv(l, col))

        if sys_char in ('G','E','C') and len(brd) >= 28:
            week_toc, tow_toc = unix_to_gps(toc_unix)
            toe_tow = brd[9]   # toe tow from RINEX
            ephems.append(dict(
                sys=sys_char, prn=prn,
                toc_unix=toc_unix, toc_week=week_toc, toc_tow=tow_toc,
                toe_week=week_toc, toe_tow=toe_tow,
                af0=af0, af1=af1, af2=af2,
                iode=int(brd[1]),  crs=brd[2],   delta_n=brd[3],  M0=brd[4],
                cuc=brd[5],   e=brd[6],     cus=brd[7],   sqrtA=brd[8],
                cic=brd[10],  OMG0=brd[11], cis=brd[12],  i0=brd[13],
                crc=brd[14],  omg=brd[15],  OMG_dot=brd[16], i_dot=brd[17],
                week=int(brd[19]) if len(brd)>19 else week_toc,
                health=int(brd[21]) if len(brd)>21 else 0,
                tgd0=brd[25] if len(brd)>25 else 0.0,
                iodc=int(brd[26]) if len(brd)>26 else 0,
            ))
        elif sys_char == 'R' and len(brd) >= 12:
            week_toc, tow_toc = unix_to_gps(toc_unix)
            ephems.append(dict(
                sys='R', prn=prn,
                toc_unix=toc_unix, toc_week=week_toc, toc_tow=tow_toc,
                freqo=int(brd[1]), health=int(brd[3]),
                pos_x=brd[4]*1e3, pos_y=brd[8]*1e3, pos_z=brd[12]*1e3 if len(brd)>12 else 0,
                vel_x=brd[5]*1e3, vel_y=brd[9]*1e3, vel_z=brd[13]*1e3 if len(brd)>13 else 0,
                acc_x=brd[6]*1e3, acc_y=brd[10]*1e3, acc_z=brd[14]*1e3 if len(brd)>14 else 0,
                tau_n=-af0, gamma=af1, delta_tau_n=0.0,
            ))
        i += 1
    return ephems

# ── 主程序 ────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--obs',  required=True)
    ap.add_argument('--nav',  required=True)
    ap.add_argument('--out',  default='gnss_urbannav.bag')
    ap.add_argument('--lat',  type=float, default=22.3)
    ap.add_argument('--lon',  type=float, default=114.2)
    ap.add_argument('--alt',  type=float, default=50.0)
    args = ap.parse_args()

    from rosbags.rosbag1 import Writer
    from rosbags.typesys import get_typestore, get_types_from_msg, Stores

    # ── typestore ──────────────────────────────────────────────────────────────
    for p in ['/root/gnss_ws/src/gnss_comm/msg',
              '/root/gnss_ws/devel/share/gnss_comm/msg',
              '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/gnss_comm/msg']:
        if Path(p).exists():
            GNSS_COMM = Path(p); break
    else:
        print('[错误] 找不到 gnss_comm msg，请确认 del1RTK 已编译'); sys.exit(1)

    add_types = {}
    for mf in GNSS_COMM.glob('*.msg'):
        try:
            add_types.update(get_types_from_msg(
                mf.read_text(), f'gnss_comm/msg/{mf.stem}'))
        except Exception as ex:
            pass

    ts = get_typestore(Stores.ROS1_NOETIC)
    ts.register(add_types)
    T = ts.types

    GnssMeasMsg     = T['gnss_comm/msg/GnssMeasMsg']
    GnssObsMsg      = T['gnss_comm/msg/GnssObsMsg']
    GnssTimeMsg     = T['gnss_comm/msg/GnssTimeMsg']
    GnssEphemMsg    = T['gnss_comm/msg/GnssEphemMsg']
    GnssGloEphemMsg = T['gnss_comm/msg/GnssGloEphemMsg']
    Header          = T['std_msgs/msg/Header']
    RosTime         = T['builtin_interfaces/msg/Time']
    NavSatFix       = T['sensor_msgs/msg/NavSatFix']

    def make_gps_time(unix_t):
        week, tow = unix_to_gps(unix_t)
        return GnssTimeMsg(week=np.uint32(week), tow=np.float64(tow))

    # ── 解析 ──────────────────────────────────────────────────────────────────
    print(f'[1/3] 解析 obs: {args.obs}')
    epochs = parse_obs(args.obs)
    print(f'      找到 {len(epochs)} 个历元')

    print(f'[2/3] 解析 nav: {args.nav}')
    ephems = parse_nav(args.nav)
    gps_ephems = [e for e in ephems if e['sys'] in ('G','E','C')]
    glo_ephems = [e for e in ephems if e['sys'] == 'R']
    print(f'      GPS/GAL/BDS 星历: {len(gps_ephems)}  GLO 星历: {len(glo_ephems)}')

    # ── 写 bag ────────────────────────────────────────────────────────────────
    out_path = Path(args.out)
    if out_path.exists(): out_path.unlink()

    psr_cols = ('C1C','C1P','C1W','C1X')
    snr_cols = ('S1C','S1P','S1W','S1X')
    dop_cols = ('D1C','D1P','D1W','D1X')
    cp_cols  = ('L1C','L1P','L1W','L1X')

    print(f'[3/3] 写入 bag: {args.out}')
    n_meas = 0

    with Writer(str(out_path)) as writer:
        c_meas = writer.add_connection('/ublox_driver/range_meas',
                    'gnss_comm/msg/GnssMeasMsg', typestore=ts)
        c_gps  = writer.add_connection('/ublox_driver/ephem',
                    'gnss_comm/msg/GnssEphemMsg', typestore=ts)
        c_glo  = writer.add_connection('/ublox_driver/glo_ephem',
                    'gnss_comm/msg/GnssGloEphemMsg', typestore=ts)
        c_lla  = writer.add_connection('/ublox_driver/receiver_lla',
                    'sensor_msgs/msg/NavSatFix', typestore=ts)

        # 先写所有星历
        ephem_seen = set()
        for ep in ephems:
            key = f"{ep['sys']}{ep['prn']}_{int(ep['toc_unix'])}"
            if key in ephem_seen: continue
            ephem_seen.add(key)
            ros_ts = int(ep['toc_unix'] * 1e9)
            sn = sat_no(ep['sys'], ep['prn'])
            gt = make_gps_time(ep['toc_unix'])

            if ep['sys'] == 'R':
                msg = GnssGloEphemMsg(
                    sat=np.uint32(sn),
                    ttr=gt, toe=gt,
                    freqo=np.int32(ep.get('freqo', 0)),
                    iode=np.uint32(0),
                    health=np.uint32(ep.get('health', 0)),
                    age=np.uint32(0),
                    ura=np.float64(1.0),
                    pos_x=np.float64(ep.get('pos_x', 0)),
                    pos_y=np.float64(ep.get('pos_y', 0)),
                    pos_z=np.float64(ep.get('pos_z', 0)),
                    vel_x=np.float64(ep.get('vel_x', 0)),
                    vel_y=np.float64(ep.get('vel_y', 0)),
                    vel_z=np.float64(ep.get('vel_z', 0)),
                    acc_x=np.float64(ep.get('acc_x', 0)),
                    acc_y=np.float64(ep.get('acc_y', 0)),
                    acc_z=np.float64(ep.get('acc_z', 0)),
                    tau_n=np.float64(ep.get('tau_n', 0)),
                    gamma=np.float64(ep.get('gamma', 0)),
                    delta_tau_n=np.float64(0))
                writer.write(c_glo, ros_ts,
                    ts.serialize(msg, 'gnss_comm/msg/GnssGloEphemMsg'))
            else:
                sqrtA = ep['sqrtA']
                A     = sqrtA * sqrtA   # 半长轴 A = sqrtA^2
                msg = GnssEphemMsg(
                    sat=np.uint32(sn),
                    ttr=gt,
                    toe=GnssTimeMsg(week=np.uint32(ep['toe_week']),
                                    tow=np.float64(ep['toe_tow'])),
                    toc=GnssTimeMsg(week=np.uint32(ep['toc_week']),
                                    tow=np.float64(ep['toc_tow'])),
                    toe_tow=np.float64(ep['toe_tow']),
                    week=np.uint32(ep['week']),
                    iode=np.uint32(ep['iode']),
                    iodc=np.uint32(ep.get('iodc', 0)),
                    health=np.uint32(ep.get('health', 0)),
                    code=np.uint32(0),
                    ura=np.float64(2.0),
                    A=np.float64(A),
                    e=np.float64(ep['e']),
                    i0=np.float64(ep['i0']),
                    omg=np.float64(ep['omg']),
                    OMG0=np.float64(ep['OMG0']),
                    M0=np.float64(ep['M0']),
                    delta_n=np.float64(ep['delta_n']),
                    OMG_dot=np.float64(ep['OMG_dot']),
                    i_dot=np.float64(ep['i_dot']),
                    cuc=np.float64(ep['cuc']), cus=np.float64(ep['cus']),
                    crc=np.float64(ep['crc']), crs=np.float64(ep['crs']),
                    cic=np.float64(ep['cic']), cis=np.float64(ep['cis']),
                    af0=np.float64(ep['af0']),
                    af1=np.float64(ep['af1']),
                    af2=np.float64(ep['af2']),
                    tgd0=np.float64(ep.get('tgd0', 0)),
                    tgd1=np.float64(0),
                    A_dot=np.float64(0),
                    n_dot=np.float64(0))
                writer.write(c_gps, ros_ts,
                    ts.serialize(msg, 'gnss_comm/msg/GnssEphemMsg'))

        # 写测量值
        for idx, (t, sv_data) in enumerate(epochs):
            ros_ts = int(t * 1e9)
            gps_t  = make_gps_time(t)
            sec_i  = int(t); nsec_i = int((t % 1) * 1e9)
            hdr    = Header(stamp=RosTime(sec=sec_i, nanosec=nsec_i), frame_id='')

            obs_list = []
            for sv, vals in sv_data.items():
                sys_char = sv[0]
                try: prn = int(sv[1:])
                except: continue
                sn = sat_no(sys_char, prn)
                if sn == 0: continue

                psr  = next((vals[k] for k in psr_cols if k in vals and not np.isnan(vals[k]) and vals[k]>1e4), None)
                if psr is None: continue

                snr  = next((vals[k] for k in snr_cols if k in vals and not np.isnan(vals[k])), 0.0)
                dopp = next((vals[k] for k in dop_cols if k in vals and not np.isnan(vals[k])), 0.0)
                cp   = next((vals[k] for k in cp_cols  if k in vals and not np.isnan(vals[k])), 0.0)
                freq = l1_freq(sys_char)
                status = np.uint8(0x01)  # psr valid

                obs_list.append(GnssObsMsg(
                    time=gps_t,
                    sat=np.uint32(sn),
                    freqs=np.array([freq], dtype=np.float64),
                    CN0=np.array([snr], dtype=np.float64),
                    LLI=np.array([0], dtype=np.uint8),
                    code=np.array([0], dtype=np.uint8),
                    psr=np.array([psr], dtype=np.float64),
                    psr_std=np.array([1.0], dtype=np.float64),
                    cp=np.array([cp], dtype=np.float64),
                    cp_std=np.array([0.003], dtype=np.float64),
                    dopp=np.array([dopp], dtype=np.float64),
                    dopp_std=np.array([1.0], dtype=np.float64),
                    status=np.array([status], dtype=np.uint8),
                ))

            if not obs_list: i += 1; continue

            msg = GnssMeasMsg(meas=obs_list)
            writer.write(c_meas, ros_ts,
                ts.serialize(msg, 'gnss_comm/msg/GnssMeasMsg'))

            fix = NavSatFix(header=hdr,
                latitude=np.float64(args.lat), longitude=np.float64(args.lon),
                altitude=np.float64(args.alt),
                position_covariance=np.zeros(9, dtype=np.float64),
                position_covariance_type=np.uint8(0))
            writer.write(c_lla, ros_ts,
                ts.serialize(fix, 'sensor_msgs/msg/NavSatFix'))
            n_meas += 1

            if (idx+1) % 50 == 0 or idx == len(epochs)-1:
                print(f'\r      {idx+1}/{len(epochs)} 历元，{n_meas} 条有效', end='', flush=True)

    print(f'\n完成！输出: {args.out}')
    print(f'  测量历元: {n_meas}，星历: {len(ephem_seen)}')
    print(f'下一步:')
    print(f'  source /root/gnss_ws/devel/setup.bash')
    print(f'  roslaunch del1RTK eval_spp.launch bag:={args.out} rviz:=false')

if __name__ == '__main__':
    main()
