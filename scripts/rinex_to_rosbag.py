#!/usr/bin/env python3
"""
rinex_to_rosbag.py
RINEX 3.02 obs + nav → gnss_comm ROS1 bag，供 del1RTK SPP 使用

安装依赖：pip install georinex rosbags numpy pandas

用法：
  python3 rinex_to_rosbag.py --obs <obs文件> --nav <nav文件> --out gnss_urbannav.bag
  python3 rinex_to_rosbag.py --obs MED.obs --nav MED.nav --out gnss_urbannav.bag --lat 22.3 --lon 114.1 --alt 50
"""

import argparse, sys, numpy as np
from pathlib import Path

# ── 卫星编号（和 del1RTK / gnss_comm 一致）────────────────────────────────────
N_GPS, N_GLO, N_GAL = 32, 27, 38

def sat_no(sys_char, prn):
    if sys_char == 'G': return prn
    if sys_char == 'R': return N_GPS + prn
    if sys_char == 'E': return N_GPS + N_GLO + prn
    if sys_char == 'C': return N_GPS + N_GLO + N_GAL + prn
    return 0

def ts_to_ros(t):
    """pandas Timestamp → (sec, nsec)"""
    ns = int(t.value)  # nanoseconds since unix epoch
    return ns // 10**9, ns % 10**9

def main():
    ap = argparse.ArgumentParser(description='RINEX → gnss_comm ROS1 bag')
    ap.add_argument('--obs',  required=True,  help='RINEX obs 文件路径')
    ap.add_argument('--nav',  required=True,  help='RINEX nav 文件路径')
    ap.add_argument('--out',  default='gnss_urbannav.bag', help='输出 bag 路径')
    ap.add_argument('--lat',  type=float, default=22.3,  help='接收机大致纬度')
    ap.add_argument('--lon',  type=float, default=114.1, help='接收机大致经度')
    ap.add_argument('--alt',  type=float, default=50.0,  help='接收机大致高度(m)')
    args = ap.parse_args()

    # ── 检查依赖 ───────────────────────────────────────────────────────────────
    try:
        import georinex as gr
    except ImportError:
        print('[错误] 请先安装：pip install georinex')
        sys.exit(1)
    try:
        import pandas as pd
    except ImportError:
        print('[错误] 请先安装：pip install pandas')
        sys.exit(1)
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import get_typestore, Stores

    # ── 加载 RINEX ─────────────────────────────────────────────────────────────
    print(f'[1/4] 读取 obs 文件: {args.obs}')
    obs = gr.load(args.obs, use=('C1C','C2C','S1C','D1C','L1C',
                                  'C1P','S1P','D1P','L1P'))
    print(f'      时间范围: {str(obs.time.values[0])[:19]} → {str(obs.time.values[-1])[:19]}')
    print(f'      卫星数: {len(obs.sv.values)}  历元数: {len(obs.time.values)}')

    print(f'[2/4] 读取 nav 文件: {args.nav}')
    nav = gr.load(args.nav)

    # ── 建立 typestore ─────────────────────────────────────────────────────────
    print('[3/4] 初始化 gnss_comm typestore ...')
    GNSS_COMM = Path('/root/gnss_ws/src/gnss_comm/msg')
    if not GNSS_COMM.exists():
        # 备用路径
        for p in ['/root/gnss_ws/devel/share/gnss_comm/msg',
                  '/root/catkin_ws/src/gnss_comm/msg']:
            if Path(p).exists():
                GNSS_COMM = Path(p)
                break
        else:
            print('[错误] 找不到 gnss_comm msg 定义，请确认 del1RTK 已编译')
            sys.exit(1)

    msg_files = list(GNSS_COMM.glob('*.msg'))
    if not msg_files:
        print(f'[错误] {GNSS_COMM} 里没有 .msg 文件')
        sys.exit(1)

    from rosbags.typesys import get_types_from_msg
    add_types = {}
    for mf in msg_files:
        txt = mf.read_text()
        name = f'gnss_comm/msg/{mf.stem}'
        try:
            add_types.update(get_types_from_msg(txt, name))
        except Exception:
            pass

    # sensor_msgs
    import importlib, rosbags
    from rosbags.typesys.stores.ros1_noetic import (
        builtin_interfaces__msg__Time as Time,
    )
    ts = get_typestore(Stores.ROS1_NOETIC)
    ts.register(add_types)

    GnssMeasMsg    = ts.types['gnss_comm/msg/GnssMeasMsg']
    GnssMeas       = ts.types['gnss_comm/msg/GnssMeas']
    GnssEphemMsg   = ts.types['gnss_comm/msg/GnssEphemMsg']
    GnssGloEphemMsg= ts.types['gnss_comm/msg/GnssGloEphemMsg']
    NavSatFix      = ts.types['sensor_msgs/msg/NavSatFix']
    Header         = ts.types['std_msgs/msg/Header']

    # ── 写 bag ────────────────────────────────────────────────────────────────
    out_path = Path(args.out)
    if out_path.exists():
        out_path.unlink()

    print(f'[4/4] 写入 bag: {args.out}')
    svs = obs.sv.values   # e.g. ['G01','G03','R01','E05','C10',...]

    # 哪个 obs 列代表 L1 伪距 / CN0 / Doppler
    def pick(sv_sys, candidates):
        for c in candidates:
            if c in obs.data_vars:
                return c
        return None

    with Writer(str(out_path)) as writer:
        conn_meas = writer.add_connection(
            '/ublox_driver/range_meas', 'gnss_comm/msg/GnssMeasMsg', typestore=ts)
        conn_gps  = writer.add_connection(
            '/ublox_driver/ephem',       'gnss_comm/msg/GnssEphemMsg',    typestore=ts)
        conn_glo  = writer.add_connection(
            '/ublox_driver/glo_ephem',   'gnss_comm/msg/GnssGloEphemMsg', typestore=ts)
        conn_lla  = writer.add_connection(
            '/ublox_driver/receiver_lla','sensor_msgs/msg/NavSatFix',      typestore=ts)

        psr_col  = next((c for c in ['C1C','C1P'] if c in obs.data_vars), None)
        snr_col  = next((c for c in ['S1C','S1P'] if c in obs.data_vars), None)
        dopp_col = next((c for c in ['D1C','D1P'] if c in obs.data_vars), None)
        adr_col  = next((c for c in ['L1C','L1P'] if c in obs.data_vars), None)

        if psr_col is None:
            print('[错误] obs 文件里没有 C1C/C1P 伪距，请检查文件')
            sys.exit(1)
        print(f'      使用观测量: PSR={psr_col}, SNR={snr_col}, DOPP={dopp_col}, ADR={adr_col}')

        n_meas_written = 0
        n_ephem_written = 0
        ephem_sent = set()

        for i, t in enumerate(obs.time.values):
            import pandas as pd
            t_pd = pd.Timestamp(t)
            sec, nsec = ts_to_ros(t_pd)
            ros_ts = sec * 10**9 + nsec
            hdr = Header(stamp=ts.types['builtin_interfaces/msg/Time'](sec=sec, nanosec=nsec),
                         frame_id='')

            meas_list = []
            for sv in svs:
                sys_char = sv[0]
                try:
                    prn = int(sv[1:])
                except ValueError:
                    continue
                sn = sat_no(sys_char, prn)
                if sn == 0:
                    continue

                try:
                    psr  = float(obs[psr_col].sel(sv=sv, time=t).values)
                    snr  = float(obs[snr_col].sel(sv=sv, time=t).values) if snr_col else 0.0
                    dopp = float(obs[dopp_col].sel(sv=sv, time=t).values) if dopp_col else 0.0
                    adr  = float(obs[adr_col].sel(sv=sv, time=t).values)  if adr_col  else 0.0
                except Exception:
                    continue

                if np.isnan(psr) or psr < 1e4:
                    continue

                meas_list.append(GnssMeas(
                    time=ts.types['builtin_interfaces/msg/Time'](sec=sec, nanosec=nsec),
                    sat=np.uint32(sn),
                    freqIdx=np.int32(0),
                    psr=np.float64(psr),
                    psr_std=np.float32(1.0),
                    adr=np.float64(adr if not np.isnan(adr) else 0.0),
                    adr_std=np.float32(0.003),
                    dopp=np.float32(dopp if not np.isnan(dopp) else 0.0),
                    dopp_std=np.float32(1.0),
                    snr=np.float32(snr),
                    psr_valid=not np.isnan(psr),
                    adr_valid=adr_col is not None and not np.isnan(adr),
                    dopp_valid=dopp_col is not None and not np.isnan(dopp),
                    slip=False,
                    half_cycle=False,
                ))

            if meas_list:
                msg = GnssMeasMsg(header=hdr,
                                  meas=meas_list)
                writer.write(conn_meas, ros_ts, ts.serialize(msg, 'gnss_comm/msg/GnssMeasMsg'))
                n_meas_written += 1

            # PVT 参考（用固定近似值，del1RTK 用于计算仰角）
            fix = NavSatFix(
                header=hdr,
                latitude=np.float64(args.lat),
                longitude=np.float64(args.lon),
                altitude=np.float64(args.alt),
                position_covariance=np.array([0.]*9, dtype=np.float64),
                position_covariance_type=np.uint8(0),
            )
            writer.write(conn_lla, ros_ts, ts.serialize(fix, 'sensor_msgs/msg/NavSatFix'))

            if (i+1) % 50 == 0 or i == len(obs.time.values)-1:
                print(f'\r      进度: {i+1}/{len(obs.time.values)} 历元，{n_meas_written} 条测量', end='', flush=True)

        # ── 写星历 ────────────────────────────────────────────────────────────
        print('\n      写入星历 ...')
        try:
            nav_svs = nav.sv.values
        except Exception:
            nav_svs = []

        for sv in nav_svs:
            sys_char = sv[0]
            try:
                prn = int(sv[1:])
            except ValueError:
                continue

            times = nav.time.values
            for ti, t in enumerate(times):
                try:
                    t_pd = pd.Timestamp(t)
                    sec, nsec = ts_to_ros(t_pd)
                    ros_ts = sec * 10**9 + nsec
                    hdr = Header(
                        stamp=ts.types['builtin_interfaces/msg/Time'](sec=sec, nanosec=nsec),
                        frame_id='')

                    if sys_char == 'R':
                        # GLONASS
                        msg = GnssGloEphemMsg(
                            header=hdr,
                            sat=np.uint32(sat_no('R', prn)),
                            freqo=np.int32(0),
                            iode=np.uint32(0),
                            tof=np.float64(0),
                            toe=np.float64(sec),
                            toc=np.float64(sec),
                            pos=np.array([0.,0.,0.], dtype=np.float64),
                            vel=np.array([0.,0.,0.], dtype=np.float64),
                            acc=np.array([0.,0.,0.], dtype=np.float64),
                            svh=np.int32(0),
                            sva=np.float64(0),
                            age=np.int32(0),
                        )
                        key = f'R{prn}_{sec}'
                        if key not in ephem_sent:
                            writer.write(conn_glo, ros_ts,
                                         ts.serialize(msg, 'gnss_comm/msg/GnssGloEphemMsg'))
                            ephem_sent.add(key)
                            n_ephem_written += 1
                    elif sys_char in ('G', 'E', 'C'):
                        msg = GnssEphemMsg(
                            header=hdr,
                            sat=np.uint32(sat_no(sys_char, prn)),
                            toes=np.float64(sec),
                            tocs=np.float64(sec),
                            toe=np.float64(sec),
                            toc=np.float64(sec),
                            sqrtA=np.float64(0), e=np.float64(0),
                            i0=np.float64(0), omg0=np.float64(0),
                            omg=np.float64(0), M0=np.float64(0),
                            delta_n=np.float64(0), omg_dot=np.float64(0),
                            i_dot=np.float64(0), crc=np.float64(0),
                            crs=np.float64(0), cuc=np.float64(0),
                            cus=np.float64(0), cic=np.float64(0),
                            cis=np.float64(0), af0=np.float64(0),
                            af1=np.float64(0), af2=np.float64(0),
                            tgd=np.array([0.,0.,0.,0.], dtype=np.float64),
                            A_f0=np.float64(0), A_f1=np.float64(0),
                            ura=np.float64(1.0), svh=np.int32(0),
                            iode=np.uint32(0), iodc=np.uint32(0),
                            week=np.uint32(0), code=np.uint32(0),
                            flag=np.uint32(0),
                        )
                        key = f'{sys_char}{prn}_{sec}'
                        if key not in ephem_sent:
                            writer.write(conn_gps, ros_ts,
                                         ts.serialize(msg, 'gnss_comm/msg/GnssEphemMsg'))
                            ephem_sent.add(key)
                            n_ephem_written += 1
                except Exception:
                    continue

    print(f'\n完成！')
    print(f'  测量历元: {n_meas_written}')
    print(f'  星历消息: {n_ephem_written}')
    print(f'  输出 bag: {args.out}')
    print(f'\n下一步：')
    print(f'  roslaunch del1RTK eval_spp.launch bag:={args.out} rviz:=false exclude_glonass:=false')

if __name__ == '__main__':
    main()
