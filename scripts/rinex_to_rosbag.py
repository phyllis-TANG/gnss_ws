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

# ── 卫星编号（和 del1RTK / gnss_comm 一致）────────────────────────────────────
N_GPS, N_GLO, N_GAL = 32, 27, 38

def sat_no(sys_char, prn):
    if sys_char == 'G': return prn
    if sys_char == 'R': return N_GPS + prn
    if sys_char == 'E': return N_GPS + N_GLO + prn
    if sys_char == 'C': return N_GPS + N_GLO + N_GAL + prn
    return 0

def rinex_epoch_to_unix(year, month, day, hour, minute, sec):
    dt = datetime(year, month, day, hour, minute, int(sec),
                  int((sec % 1) * 1e6), tzinfo=timezone.utc)
    return dt.timestamp()

# ── RINEX 3 obs 解析 ───────────────────────────────────────────────────────────
def parse_obs(filepath):
    """
    返回 list of (unix_time, {sv_str: {obs_type: value}})
    sv_str 例如 'G01', 'R03', 'E05', 'C12'
    """
    lines = Path(filepath).read_text(errors='ignore').splitlines()

    # 读表头：各系统的观测量类型
    sys_obs_types = {}   # {'G': ['C1C','S1C',...], 'R': [...], ...}
    in_header = True
    i = 0
    while i < len(lines) and in_header:
        line = lines[i]
        label = line[60:].strip() if len(line) > 60 else ''
        if 'SYS / # / OBS TYPES' in label:
            sys_char = line[0]
            n = int(line[3:6])
            types = line[7:60].split()
            # 可能跨多行
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
        # 解析历元行
        try:
            year  = int(line[2:6])
            month = int(line[7:9])
            day   = int(line[10:12])
            hour  = int(line[13:15])
            minute= int(line[16:18])
            sec   = float(line[19:29])
            n_sv  = int(line[32:35])
        except Exception:
            i += 1
            continue

        t = rinex_epoch_to_unix(year, month, day, hour, minute, sec)
        sv_data = {}

        for j in range(n_sv):
            i += 1
            if i >= len(lines):
                break
            sv_line = lines[i]
            if len(sv_line) < 3:
                continue
            sv = sv_line[:3]          # e.g. 'G01'
            sys_char = sv[0]
            if sys_char not in sys_obs_types:
                continue
            obs_types = sys_obs_types[sys_char]
            obs_vals = {}
            for k, ot in enumerate(obs_types):
                start = 3 + k * 16
                end   = start + 14
                raw = sv_line[start:end].strip() if len(sv_line) > start else ''
                try:
                    obs_vals[ot] = float(raw)
                except ValueError:
                    obs_vals[ot] = float('nan')
            sv_data[sv] = obs_vals

        epochs.append((t, sv_data))
        i += 1

    return epochs

# ── RINEX 3/2 nav 解析（GPS / GAL / BDS）────────────────────────────────────
def parse_nav(filepath):
    """
    返回 list of dict，每个是一颗卫星的一组星历参数
    """
    lines = Path(filepath).read_text(errors='ignore').splitlines()

    # 判断 RINEX 版本
    version = 3
    is_mixed = False
    for line in lines[:20]:
        if 'RINEX VERSION' in line[60:]:
            try:
                version = int(float(line[:9]))
            except Exception:
                pass
            if 'M' in line[20:21]:
                is_mixed = True
        if 'END OF HEADER' in line[60:]:
            break

    ephems = []
    i = 0
    # 跳过表头
    while i < len(lines):
        if 'END OF HEADER' in (lines[i][60:] if len(lines[i]) > 60 else ''):
            i += 1
            break
        i += 1

    def read_val(line, col):
        s = line[col:col+19].strip().replace('D','e').replace('d','e')
        try:
            return float(s)
        except Exception:
            return 0.0

    while i < len(lines):
        line = lines[i]
        if not line or line[0] == ' ':
            i += 1
            continue

        # RINEX 3: 系统字符在第0位
        # RINEX 2: 只有 GPS，PRN 在前2位
        try:
            if version >= 3:
                sys_char = line[0]
                prn = int(line[1:3])
                year = int(line[4:8])
                month= int(line[9:11])
                day  = int(line[12:14])
                hour = int(line[15:17])
                minute=int(line[18:20])
                sec  = float(line[21:23])
                af0  = read_val(line, 23)
                af1  = read_val(line, 42)
                af2  = read_val(line, 61)
            else:  # RINEX 2
                sys_char = 'G'
                prn  = int(line[0:2])
                year = int(line[3:5]) + 2000
                month= int(line[6:8])
                day  = int(line[9:11])
                hour = int(line[12:14])
                minute=int(line[15:17])
                sec  = float(line[17:22])
                af0  = read_val(line, 22)
                af1  = read_val(line, 41)
                af2  = read_val(line, 60)
        except Exception:
            i += 1
            continue

        if sys_char not in ('G', 'E', 'C', 'R'):
            i += 8 if version >= 3 else 8
            i += 1
            continue

        toc = rinex_epoch_to_unix(year, month, day, hour, minute, sec)

        # 读后续广播轨道参数行
        brd = []
        n_lines = 7 if sys_char != 'R' else 3
        for _ in range(n_lines):
            i += 1
            if i >= len(lines):
                break
            l = lines[i]
            for col in (4, 23, 42, 61):
                brd.append(read_val(l, col))

        if sys_char == 'G' and len(brd) >= 28:
            ephems.append(dict(
                sys='G', prn=prn, toc=toc,
                af0=af0, af1=af1, af2=af2,
                iode=brd[1],  crs=brd[2],  delta_n=brd[3], M0=brd[4],
                cuc=brd[5],   e=brd[6],    cus=brd[7],      sqrtA=brd[8],
                toe=brd[9],   cic=brd[10], omg0=brd[11],    cis=brd[12],
                i0=brd[13],   crc=brd[14], omg=brd[15],     omg_dot=brd[16],
                i_dot=brd[17],week=brd[19],svh=int(brd[21]),tgd=brd[25],
                iodc=brd[26],
            ))
        elif sys_char in ('E', 'C') and len(brd) >= 20:
            ephems.append(dict(
                sys=sys_char, prn=prn, toc=toc,
                af0=af0, af1=af1, af2=af2,
                iode=brd[1],  crs=brd[2],  delta_n=brd[3], M0=brd[4],
                cuc=brd[5],   e=brd[6],    cus=brd[7],      sqrtA=brd[8],
                toe=brd[9],   cic=brd[10], omg0=brd[11],    cis=brd[12],
                i0=brd[13],   crc=brd[14], omg=brd[15],     omg_dot=brd[16],
                i_dot=brd[17],week=brd[18] if len(brd)>18 else 0,
                svh=0, tgd=0, iodc=0,
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
              '/root/gnss_ws/devel/share/gnss_comm/msg']:
        if Path(p).exists():
            GNSS_COMM = Path(p); break
    else:
        print('[错误] 找不到 gnss_comm msg，请确认 del1RTK 已编译')
        sys.exit(1)

    add_types = {}
    for mf in GNSS_COMM.glob('*.msg'):
        try:
            add_types.update(get_types_from_msg(
                mf.read_text(), f'gnss_comm/msg/{mf.stem}'))
        except Exception:
            pass

    ts = get_typestore(Stores.ROS1_NOETIC)
    ts.register(add_types)

    T   = ts.types
    Hdr = T['std_msgs/msg/Header']
    Ros_Time = T['builtin_interfaces/msg/Time']
    GnssMeasMsg    = T['gnss_comm/msg/GnssMeasMsg']
    GnssMeas       = T['gnss_comm/msg/GnssMeas']
    GnssEphemMsg   = T['gnss_comm/msg/GnssEphemMsg']
    GnssGloEphemMsg= T['gnss_comm/msg/GnssGloEphemMsg']
    NavSatFix      = T['sensor_msgs/msg/NavSatFix']

    def make_time(unix_t):
        sec  = int(unix_t)
        nsec = int((unix_t % 1) * 1e9)
        return Ros_Time(sec=sec, nanosec=nsec), sec * 10**9 + nsec

    # ── 解析文件 ───────────────────────────────────────────────────────────────
    print(f'[1/3] 解析 obs: {args.obs}')
    epochs = parse_obs(args.obs)
    print(f'      找到 {len(epochs)} 个历元')

    print(f'[2/3] 解析 nav: {args.nav}')
    ephems = parse_nav(args.nav)
    print(f'      找到 {len(ephems)} 条星历')

    # ── 写 bag ────────────────────────────────────────────────────────────────
    out_path = Path(args.out)
    if out_path.exists(): out_path.unlink()

    print(f'[3/3] 写入 bag: {args.out}')
    n_meas = 0
    psr_priority = ('C1C','C1P','C1W','C1X')
    snr_priority = ('S1C','S1P','S1W','S1X')
    dop_priority = ('D1C','D1P','D1W','D1X')

    with Writer(str(out_path)) as writer:
        c_meas = writer.add_connection('/ublox_driver/range_meas',
                    'gnss_comm/msg/GnssMeasMsg', typestore=ts)
        c_gps  = writer.add_connection('/ublox_driver/ephem',
                    'gnss_comm/msg/GnssEphemMsg', typestore=ts)
        c_glo  = writer.add_connection('/ublox_driver/glo_ephem',
                    'gnss_comm/msg/GnssGloEphemMsg', typestore=ts)
        c_lla  = writer.add_connection('/ublox_driver/receiver_lla',
                    'sensor_msgs/msg/NavSatFix', typestore=ts)

        # 先写星历
        ephem_written = set()
        for ep in ephems:
            rt, ros_ts = make_time(ep['toc'])
            hdr = Hdr(stamp=rt, frame_id='')
            key = f"{ep['sys']}{ep['prn']}_{int(ep['toc'])}"
            if key in ephem_written: continue
            ephem_written.add(key)
            sn = sat_no(ep['sys'], ep['prn'])
            if ep['sys'] == 'R':
                msg = GnssGloEphemMsg(
                    header=hdr, sat=np.uint32(sn), freqo=np.int32(0),
                    iode=np.uint32(0), tof=np.float64(0),
                    toe=np.float64(ep['toc']), toc=np.float64(ep['toc']),
                    pos=np.zeros(3,dtype=np.float64),
                    vel=np.zeros(3,dtype=np.float64),
                    acc=np.zeros(3,dtype=np.float64),
                    svh=np.int32(0), sva=np.float64(0), age=np.int32(0))
                writer.write(c_glo, ros_ts,
                    ts.serialize(msg,'gnss_comm/msg/GnssGloEphemMsg'))
            else:
                msg = GnssEphemMsg(
                    header=hdr, sat=np.uint32(sn),
                    toes=np.float64(ep['toe']), tocs=np.float64(ep['toc']),
                    toe=np.float64(ep['toe']),  toc=np.float64(ep['toc']),
                    sqrtA=np.float64(ep['sqrtA']), e=np.float64(ep['e']),
                    i0=np.float64(ep['i0']),       omg0=np.float64(ep['omg0']),
                    omg=np.float64(ep['omg']),     M0=np.float64(ep['M0']),
                    delta_n=np.float64(ep['delta_n']),
                    omg_dot=np.float64(ep['omg_dot']),
                    i_dot=np.float64(ep['i_dot']),
                    crc=np.float64(ep['crc']),  crs=np.float64(ep['crs']),
                    cuc=np.float64(ep['cuc']),  cus=np.float64(ep['cus']),
                    cic=np.float64(ep['cic']),  cis=np.float64(ep['cis']),
                    af0=np.float64(ep['af0']),  af1=np.float64(ep['af1']),
                    af2=np.float64(ep['af2']),
                    tgd=np.array([ep['tgd'],0,0,0],dtype=np.float64),
                    A_f0=np.float64(0), A_f1=np.float64(0),
                    ura=np.float64(2.0), svh=np.int32(int(ep['svh'])),
                    iode=np.uint32(int(ep['iode'])),
                    iodc=np.uint32(int(ep['iodc'])),
                    week=np.uint32(int(ep['week'])),
                    code=np.uint32(0), flag=np.uint32(0))
                writer.write(c_gps, ros_ts,
                    ts.serialize(msg,'gnss_comm/msg/GnssEphemMsg'))

        # 写测量值
        for idx, (t, sv_data) in enumerate(epochs):
            rt, ros_ts = make_time(t)
            hdr = Hdr(stamp=rt, frame_id='')

            meas_list = []
            for sv, obs_vals in sv_data.items():
                sys_char = sv[0]
                try: prn = int(sv[1:])
                except ValueError: continue
                sn = sat_no(sys_char, prn)
                if sn == 0: continue

                psr = next((obs_vals.get(k) for k in psr_priority
                            if obs_vals.get(k) and not np.isnan(obs_vals.get(k,float('nan')))), None)
                if psr is None or np.isnan(psr) or psr < 1e4: continue

                snr  = next((obs_vals.get(k,0) for k in snr_priority
                             if not np.isnan(obs_vals.get(k,float('nan')))), 0.0)
                dopp = next((obs_vals.get(k,0) for k in dop_priority
                             if not np.isnan(obs_vals.get(k,float('nan')))), 0.0)

                meas_list.append(GnssMeas(
                    time=rt, sat=np.uint32(sn), freqIdx=np.int32(0),
                    psr=np.float64(psr),  psr_std=np.float32(1.0),
                    adr=np.float64(0),    adr_std=np.float32(0.003),
                    dopp=np.float32(dopp),dopp_std=np.float32(1.0),
                    snr=np.float32(snr),
                    psr_valid=True, adr_valid=False,
                    dopp_valid=(dopp != 0.0),
                    slip=False, half_cycle=False))

            if not meas_list: continue

            msg = GnssMeasMsg(header=hdr, meas=meas_list)
            writer.write(c_meas, ros_ts,
                ts.serialize(msg,'gnss_comm/msg/GnssMeasMsg'))

            fix = NavSatFix(header=hdr,
                latitude=np.float64(args.lat), longitude=np.float64(args.lon),
                altitude=np.float64(args.alt),
                position_covariance=np.zeros(9,dtype=np.float64),
                position_covariance_type=np.uint8(0))
            writer.write(c_lla, ros_ts,
                ts.serialize(fix,'sensor_msgs/msg/NavSatFix'))
            n_meas += 1

            if (idx+1) % 50 == 0 or idx == len(epochs)-1:
                print(f'\r      {idx+1}/{len(epochs)} 历元，{n_meas} 条有效', end='', flush=True)

    print(f'\n完成！输出: {args.out}')
    print(f'下一步:')
    print(f'  roslaunch del1RTK eval_spp.launch bag:={args.out} rviz:=false exclude_glonass:=false')

if __name__ == '__main__':
    main()
