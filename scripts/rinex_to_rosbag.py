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
            # RINEX nav brd 索引（每行4个字段，7行共28个）：
            # ORBIT1: [0]=IODE [1]=Crs  [2]=Delta_n [3]=M0
            # ORBIT2: [4]=Cuc  [5]=e    [6]=Cus     [7]=sqrtA
            # ORBIT3: [8]=Toe  [9]=Cic  [10]=Omega0 [11]=Cis
            # ORBIT4: [12]=i0  [13]=Crc [14]=omega  [15]=Omega_dot
            # ORBIT5: [16]=i_dot [17]=CodesL2 [18]=GPS_week [19]=L2Pflag
            # ORBIT6: [20]=SV_acc [21]=health [22]=TGD [23]=IODC
            # ORBIT7: [24]=TransmitTime ...
            toe_tow = brd[8]
            ephems.append(dict(
                sys=sys_char, prn=prn,
                toc_unix=toc_unix, toc_week=week_toc, toc_tow=tow_toc,
                toe_week=week_toc, toe_tow=toe_tow,
                af0=af0, af1=af1, af2=af2,
                iode=int(brd[0]),  crs=brd[1],   delta_n=brd[2],  M0=brd[3],
                cuc=brd[4],   e=brd[5],     cus=brd[6],   sqrtA=brd[7],
                cic=brd[9],   OMG0=brd[10], cis=brd[11],  i0=brd[12],
                crc=brd[13],  omg=brd[14],  OMG_dot=brd[15], i_dot=brd[16],
                week=int(brd[18]) if len(brd)>18 else week_toc,
                health=int(brd[21]) if len(brd)>21 else 0,
                tgd0=brd[22] if len(brd)>22 else 0.0,
                iodc=int(brd[23]) if len(brd)>23 else 0,
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

    # 使用 ROS1 原生 rosbag + 编译好的消息类，确保序列化完全兼容
    try:
        import rosbag, rospy
        from gnss_comm.msg import (GnssMeasMsg, GnssObsMsg, GnssTimeMsg,
                                    GnssEphemMsg, GnssGloEphemMsg)
        from sensor_msgs.msg import NavSatFix, NavSatStatus
        from std_msgs.msg import Header
    except ImportError as e:
        print(f'[错误] 无法导入 ROS 消息: {e}')
        print('请先执行: source /root/gnss_ws/devel/setup.bash')
        sys.exit(1)

    def make_gps_time(unix_t):
        week, tow = unix_to_gps(unix_t)
        m = GnssTimeMsg()
        m.week = int(week)
        m.tow  = float(tow)
        return m

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

    first_epoch_t = epochs[0][0] if epochs else 0
    ephem_ros_t   = rospy.Time(int(first_epoch_t - 1), 0)

    with rosbag.Bag(str(out_path), 'w') as bag:
        # 先写所有星历（时间戳在第一个观测历元前1秒）
        ephem_seen = set()
        for ep in ephems:
            key = f"{ep['sys']}{ep['prn']}_{int(ep['toc_unix'])}"
            if key in ephem_seen: continue
            ephem_seen.add(key)
            sn = sat_no(ep['sys'], ep['prn'])

            if ep['sys'] == 'R':
                msg = GnssGloEphemMsg()
                msg.sat        = int(sn)
                msg.ttr        = make_gps_time(ep['toc_unix'])
                msg.toe        = make_gps_time(ep['toc_unix'])
                msg.freqo      = int(ep.get('freqo', 0))
                msg.iode       = 0
                msg.health     = int(ep.get('health', 0))
                msg.age        = 0
                msg.ura        = 1.0
                msg.pos_x      = float(ep.get('pos_x', 0))
                msg.pos_y      = float(ep.get('pos_y', 0))
                msg.pos_z      = float(ep.get('pos_z', 0))
                msg.vel_x      = float(ep.get('vel_x', 0))
                msg.vel_y      = float(ep.get('vel_y', 0))
                msg.vel_z      = float(ep.get('vel_z', 0))
                msg.acc_x      = float(ep.get('acc_x', 0))
                msg.acc_y      = float(ep.get('acc_y', 0))
                msg.acc_z      = float(ep.get('acc_z', 0))
                msg.tau_n      = float(ep.get('tau_n', 0))
                msg.gamma      = float(ep.get('gamma', 0))
                msg.delta_tau_n = 0.0
                bag.write('/ublox_driver/glo_ephem', msg, t=ephem_ros_t)
            else:
                sqrtA = ep['sqrtA']
                msg = GnssEphemMsg()
                msg.sat        = int(sn)
                msg.ttr        = make_gps_time(ep['toc_unix'])
                msg.toe        = make_gps_time(ep['toc_unix'])
                msg.toe.week   = int(ep['toe_week'])
                msg.toe.tow    = float(ep['toe_tow'])
                msg.toc        = make_gps_time(ep['toc_unix'])
                msg.toe_tow    = float(ep['toe_tow'])
                msg.week       = int(ep['week'])
                msg.iode       = int(ep['iode'])
                msg.iodc       = int(ep.get('iodc', 0))
                msg.health     = int(ep.get('health', 0))
                msg.code       = 0
                msg.ura        = 2.0
                msg.A          = float(sqrtA * sqrtA)
                msg.e          = float(ep['e'])
                msg.i0         = float(ep['i0'])
                msg.omg        = float(ep['omg'])
                msg.OMG0       = float(ep['OMG0'])
                msg.M0         = float(ep['M0'])
                msg.delta_n    = float(ep['delta_n'])
                msg.OMG_dot    = float(ep['OMG_dot'])
                msg.i_dot      = float(ep['i_dot'])
                msg.cuc        = float(ep['cuc'])
                msg.cus        = float(ep['cus'])
                msg.crc        = float(ep['crc'])
                msg.crs        = float(ep['crs'])
                msg.cic        = float(ep['cic'])
                msg.cis        = float(ep['cis'])
                msg.af0        = float(ep['af0'])
                msg.af1        = float(ep['af1'])
                msg.af2        = float(ep['af2'])
                msg.tgd0       = float(ep.get('tgd0', 0))
                msg.tgd1       = 0.0
                msg.A_dot      = 0.0
                msg.n_dot      = 0.0
                bag.write('/ublox_driver/ephem', msg, t=ephem_ros_t)

        # 写观测测量值
        for idx, (t, sv_data) in enumerate(epochs):
            ros_t = rospy.Time(int(t), int((t % 1) * 1e9))
            gps_t = make_gps_time(t)

            obs_list = []
            for sv, vals in sv_data.items():
                sys_char = sv[0]
                try: prn = int(sv[1:])
                except: continue
                sn = sat_no(sys_char, prn)
                if sn == 0: continue

                psr = next((vals[k] for k in psr_cols if k in vals and not np.isnan(vals[k]) and vals[k]>1e4), None)
                if psr is None: continue

                snr  = next((vals[k] for k in snr_cols if k in vals and not np.isnan(vals[k])), 0.0)
                dopp = next((vals[k] for k in dop_cols if k in vals and not np.isnan(vals[k])), 0.0)
                cp   = next((vals[k] for k in cp_cols  if k in vals and not np.isnan(vals[k])), 0.0)

                obs = GnssObsMsg()
                obs.time      = gps_t
                obs.sat       = int(sn)
                obs.freqs     = [float(l1_freq(sys_char))]
                obs.CN0       = [float(snr)]
                obs.LLI       = [0]
                obs.code      = [0]
                obs.psr       = [float(psr)]
                obs.psr_std   = [1.0]
                obs.cp        = [float(cp)]
                obs.cp_std    = [0.003]
                obs.dopp      = [float(dopp)]
                obs.dopp_std  = [1.0]
                obs.status    = [1]
                obs_list.append(obs)

            if not obs_list:
                continue

            meas = GnssMeasMsg()
            meas.meas = obs_list
            bag.write('/ublox_driver/range_meas', meas, t=ros_t)

            fix = NavSatFix()
            fix.header.seq       = 0
            fix.header.stamp     = ros_t
            fix.header.frame_id  = ''
            fix.status.status    = 0
            fix.status.service   = 1
            fix.latitude         = float(args.lat)
            fix.longitude        = float(args.lon)
            fix.altitude         = float(args.alt)
            fix.position_covariance = [0.0] * 9
            fix.position_covariance_type = 0
            bag.write('/ublox_driver/receiver_lla', fix, t=ros_t)
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
