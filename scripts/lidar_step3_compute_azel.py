#!/usr/bin/env python3
"""
lidar_step3_compute_azel.py
对每个 GNSS 历元的可见卫星，计算方位角 (azimuth) 和仰角 (elevation)。
接收机位置来自 NovAtel INSPVAX 轨迹（novatel_trajectory.csv）。

输出 epoch_sat_azel.csv，供 Step 4 射线追踪使用。

用法（宿主机或容器均可，需要 numpy）：
  python3 lidar_step3_compute_azel.py \
    --obs  /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
    --nav  /root/urbannav_gnss/hksc137c.21n \
    --traj /root/novatel_trajectory.csv \
    --out  /root/epoch_sat_azel.csv \
    --min_elev 5.0

时间处理说明：
  read_rinex_obs 把 GPS 历元当 UTC 解析，导致 epoch.time_unix 比真实 UTC 多 18 秒。
  compute_sat_position 内部 unix_to_gpst 同样少加 18 秒，两者恰好抵消 → 卫星位置正确。
  但与 INSPVAX 轨迹（真实 UTC unix 时间）匹配时，需减去 LEAP_SECONDS=18。
"""

import argparse, csv, math, sys, os
import numpy as np

# 复用 del2AINLOS 中的 rinex_utils
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RINEX_UTILS = os.path.join(SCRIPT_DIR,
    '../src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts')
# 备用路径（兼容不同目录结构）
_RINEX_UTILS_ALT = os.path.join(SCRIPT_DIR,
    '../PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts')
for _p in [RINEX_UTILS, _RINEX_UTILS_ALT,
           '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts',
           '/root/gnss_ws/devel/lib/del2AINLOS']:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rinex_utils import (
    read_rinex_obs, read_rinex_nav,
    compute_sat_position, find_closest_ephem,
    elevation_azimuth, llh_to_ecef,
)

LEAP_SECONDS = 18  # GPS 时间 - UTC 时间（2021年）

ap = argparse.ArgumentParser()
ap.add_argument('--obs',  default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav',  default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--traj', default='/root/novatel_trajectory.csv')
ap.add_argument('--out',  default='/root/epoch_sat_azel.csv')
ap.add_argument('--min_elev', type=float, default=5.0,
                help='最小仰角过滤（度），低于此值跳过（默认5°）')
args = ap.parse_args()

# ── 读 INSPVAX 轨迹 ────────────────────────────────────────────────
print('读取 INSPVAX 轨迹...')
traj = []
with open(args.traj) as f:
    for row in csv.DictReader(f):
        traj.append((
            float(row['unix_t']),
            float(row['lat']), float(row['lon']), float(row['alt_m'])
        ))
traj.sort(key=lambda x: x[0])
traj_times = np.array([r[0] for r in traj])
print(f'  {len(traj)} 个位姿，UTC {traj[0][0]:.1f} ~ {traj[-1][0]:.1f}')

def get_rx_ecef(utc_t):
    """最近邻插值，返回接收机 ECEF 坐标和 LLH"""
    idx = int(np.searchsorted(traj_times, utc_t))
    idx = min(max(idx, 0), len(traj) - 1)
    # 取最近邻（比较前后）
    if idx > 0 and abs(traj_times[idx-1] - utc_t) < abs(traj_times[idx] - utc_t):
        idx -= 1
    _, lat, lon, alt = traj[idx]
    ecef = llh_to_ecef(lat, lon, alt)
    return ecef, lat, lon, alt

# ── 读星历 ─────────────────────────────────────────────────────────
print('读取导航电文...')
ephem_dict = read_rinex_nav(args.nav)
total_ephem = sum(len(v) for v in ephem_dict.values())
print(f'  {len(ephem_dict)} 颗卫星，共 {total_ephem} 条星历')

# ── 读观测文件 ─────────────────────────────────────────────────────
print('读取观测文件（可能需要数秒）...')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} 个历元')

# ── 主循环：计算每颗卫星的方位角/仰角 ──────────────────────────────
print(f'\n计算 azimuth / elevation（最小仰角 {args.min_elev}°）...')

rows = []
no_ephem_sats = set()
traj_misses = 0

for ep_i, epoch in enumerate(obs_epochs):
    # RINEX unix_t 比真实 UTC 多 LEAP_SECONDS（见文件头注释）
    rinex_unix_t = epoch.time_unix          # 用于卫星位置计算（两偏差相消）
    utc_t        = rinex_unix_t - LEAP_SECONDS  # 用于匹配 INSPVAX

    # 检查 INSPVAX 时间覆盖范围
    if utc_t < traj_times[0] - 2.0 or utc_t > traj_times[-1] + 2.0:
        traj_misses += 1
        continue

    rx_ecef, rx_lat, rx_lon, rx_alt = get_rx_ecef(utc_t)

    sat_count = 0
    for obs in epoch.obs_list:
        sat_id = obs.sat_id
        sys_char = obs.sys

        # 找最近星历
        eph_list = ephem_dict.get(sat_id, [])
        eph = find_closest_ephem(eph_list, rinex_unix_t)
        if eph is None:
            no_ephem_sats.add(sat_id)
            continue

        # 计算卫星 ECEF 位置
        sat_ecef, dt_sv = compute_sat_position(eph, rinex_unix_t)
        if sat_ecef is None:
            continue

        # 计算仰角和方位角
        elev, azim = elevation_azimuth(rx_ecef, sat_ecef)

        if elev < args.min_elev:
            continue

        # 最佳伪距（L1 优先：C1C/C1P/C1X）
        psr = 0.0
        for key in ['C1C', 'C1P', 'C1X', 'C2C', 'C2P']:
            if key in obs.pseudorange:
                psr = obs.pseudorange[key]
                break
        if not psr and obs.pseudorange:
            psr = list(obs.pseudorange.values())[0]

        # 最佳 CN0
        cn0 = 0.0
        for key in ['S1C', 'S1P', 'S1X', 'S2C']:
            if key in obs.cn0:
                cn0 = obs.cn0[key]
                break
        if not cn0 and obs.cn0:
            cn0 = list(obs.cn0.values())[0]

        rows.append({
            'unix_t':       f'{rinex_unix_t:.3f}',
            'utc_t':        f'{utc_t:.3f}',
            'sat_id':       sat_id,
            'sys':          sys_char,
            'prn':          obs.prn,
            'elevation_deg': f'{elev:.4f}',
            'azimuth_deg':  f'{azim:.4f}',
            'pseudorange':  f'{psr:.3f}',
            'cn0':          f'{cn0:.2f}',
            'rx_lat':       f'{rx_lat:.8f}',
            'rx_lon':       f'{rx_lon:.8f}',
            'rx_alt':       f'{rx_alt:.3f}',
        })
        sat_count += 1

    if (ep_i + 1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} 历元，已累积 {len(rows)} 条记录', end='', flush=True)

print(f'\n完成，共 {len(rows)} 条（卫星×历元）记录')
if no_ephem_sats:
    print(f'  无星历卫星：{sorted(no_ephem_sats)}')
if traj_misses:
    print(f'  超出 INSPVAX 时间范围历元数：{traj_misses}')

# ── 写 CSV ────────────────────────────────────────────────────────
COLS = ['unix_t', 'utc_t', 'sat_id', 'sys', 'prn',
        'elevation_deg', 'azimuth_deg', 'pseudorange', 'cn0',
        'rx_lat', 'rx_lon', 'rx_alt']

with open(args.out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS)
    w.writeheader()
    w.writerows(rows)

print(f'已保存：{args.out}')

# ── 简单统计 ──────────────────────────────────────────────────────
if rows:
    elevs = [float(r['elevation_deg']) for r in rows]
    azims = [float(r['azimuth_deg'])   for r in rows]
    sats  = set(r['sat_id'] for r in rows)
    epochs_with_data = set(r['unix_t'] for r in rows)
    print(f'\n--- 统计 ---')
    print(f'历元数（有数据）: {len(epochs_with_data)}')
    print(f'卫星种类:         {len(sats)} 颗 ({sorted(sats)[:8]}...)')
    print(f'仰角范围:         {min(elevs):.1f}° ~ {max(elevs):.1f}°，均值 {np.mean(elevs):.1f}°')
    print(f'方位角范围:       {min(azims):.1f}° ~ {max(azims):.1f}°')
    sys_cnt = {}
    for r in rows:
        sys_cnt[r['sys']] = sys_cnt.get(r['sys'], 0) + 1
    print(f'星座分布: { {k: v for k, v in sorted(sys_cnt.items())} }')
