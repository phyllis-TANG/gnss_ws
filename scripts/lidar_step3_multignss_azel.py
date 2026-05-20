#!/usr/bin/env python3
"""
lidar_step3_multignss_azel.py
Multi-GNSS (GPS / BeiDou / Galileo) version of lidar_step3_compute_azel.py.

For every GNSS epoch, compute azimuth and elevation for all visible G/C/E
satellites.  Receiver position comes from the NovAtel INSPVAX trajectory
(novatel_trajectory.csv).

Output: epoch_sat_azel_multignss.csv  (same columns as the GPS-only version;
        the 'sys' column captures the constellation).

Usage (in container):
  python3 lidar_step3_multignss_azel.py \
    --obs      /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
    --nav_gps  /root/urbannav_gnss/hksc137c.21n \
    --nav_bds  /root/urbannav_gnss/hksc137c.21f \
    --nav_gal  /root/urbannav_gnss/hksc137c.21l \
    --traj     /root/novatel_trajectory.csv \
    --out      /root/epoch_sat_azel_multignss.csv \
    --min_elev 5.0

Timing note (unchanged from Step 3 GPS-only):
  read_rinex_obs parses GPS epoch times as UTC, making epoch.time_unix 18 s
  ahead of true UTC.  compute_sat_position has the matching offset, so
  satellite positions are correct.  For matching the INSPVAX trajectory
  (true UTC unix time) subtract LEAP_SECONDS=18.
"""

import argparse, csv, math, sys, os
import numpy as np

# ── rinex_utils search path ────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in [
    os.path.join(SCRIPT_DIR, '../src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts'),
    os.path.join(SCRIPT_DIR, '../PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts'),
    '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts',
    '/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts',
]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rinex_utils import (
    read_rinex_obs, read_rinex_nav,
    compute_sat_position, find_closest_ephem,
    elevation_azimuth, llh_to_ecef,
)

LEAP_SECONDS = 18  # GPS time − UTC (2021)

# System-specific pseudorange priority keys
PSR_KEYS = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
}

# ── argument parsing ───────────────────────────────────────────────────────
ap = argparse.ArgumentParser(
    description='Multi-GNSS az/el computation (G/C/E) for LiDAR NLOS pipeline')
ap.add_argument('--obs',     default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps', default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds', default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal', default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--traj',    default='/root/novatel_trajectory.csv')
ap.add_argument('--out',     default='/root/epoch_sat_azel_multignss.csv')
ap.add_argument('--min_elev', type=float, default=5.0,
                help='Elevation cutoff in degrees (default 5 deg)')
args = ap.parse_args()


# ── nav loading helper ─────────────────────────────────────────────────────
def load_nav(path, sys_prefix):
    """
    Load a RINEX nav file and remap all satellite keys to the form
    '<sys_prefix><nn>' (e.g. 'G01', 'C03', 'E12').

    Handles three key formats returned by read_rinex_nav:
      - integer  : direct PRN -> '<prefix>{:02d}'
      - string beginning with a letter but wrong prefix -> replace first char
      - string already in correct format -> keep as-is

    Returns an empty dict if the file cannot be read.
    """
    try:
        raw = read_rinex_nav(path)
    except Exception as exc:
        print(f'  [WARNING] Failed to read nav file {path}: {exc}')
        return {}

    remapped = {}
    for key, ephem_list in raw.items():
        if isinstance(key, int):
            new_key = f'{sys_prefix}{key:02d}'
        elif isinstance(key, str) and key and key[0].isalpha():
            if key[0] == sys_prefix:
                new_key = key
            else:
                new_key = sys_prefix + key[1:]
        else:
            new_key = key
        remapped[new_key] = ephem_list

    return remapped


# ── load trajectory ────────────────────────────────────────────────────────
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
    """Nearest-neighbour interpolation; returns (ecef, lat, lon, alt)."""
    idx = int(np.searchsorted(traj_times, utc_t))
    idx = min(max(idx, 0), len(traj) - 1)
    if idx > 0 and abs(traj_times[idx - 1] - utc_t) < abs(traj_times[idx] - utc_t):
        idx -= 1
    _, lat, lon, alt = traj[idx]
    ecef = llh_to_ecef(lat, lon, alt)
    return ecef, lat, lon, alt


# ── load nav files ─────────────────────────────────────────────────────────
print('读取导航电文（GPS / BeiDou / Galileo）...')
ephem_gps = load_nav(args.nav_gps, 'G')
ephem_bds = load_nav(args.nav_bds, 'C')
ephem_gal = load_nav(args.nav_gal, 'E')

ephem_dict = {}
ephem_dict.update(ephem_gps)
ephem_dict.update(ephem_bds)
ephem_dict.update(ephem_gal)

total_ephem = sum(len(v) for v in ephem_dict.values())
print(f'  GPS {len(ephem_gps)} 颗，BDS {len(ephem_bds)} 颗，'
      f'GAL {len(ephem_gal)} 颗  共 {total_ephem} 条星历')

# ── load observations ──────────────────────────────────────────────────────
print('读取观测文件（可能需要数秒）...')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} 个历元')

# ── main loop ──────────────────────────────────────────────────────────────
print(f'\n计算 azimuth / elevation（最小仰角 {args.min_elev}°，星座: G/C/E）...')

rows          = []
no_ephem_sats = set()
traj_misses   = 0

for ep_i, epoch in enumerate(obs_epochs):
    rinex_unix_t = epoch.time_unix
    utc_t        = rinex_unix_t - LEAP_SECONDS

    if utc_t < traj_times[0] - 2.0 or utc_t > traj_times[-1] + 2.0:
        traj_misses += 1
        continue

    rx_ecef, rx_lat, rx_lon, rx_alt = get_rx_ecef(utc_t)

    sat_count = 0
    for obs in epoch.obs_list:
        sat_id   = obs.sat_id
        sys_char = obs.sys

        # Only process GPS, BeiDou and Galileo
        if sys_char not in ('G', 'C', 'E'):
            continue

        # Find closest ephemeris
        eph_list = ephem_dict.get(sat_id, [])
        eph = find_closest_ephem(eph_list, rinex_unix_t)
        if eph is None:
            no_ephem_sats.add(sat_id)
            continue

        # Satellite ECEF position
        sat_ecef, dt_sv = compute_sat_position(eph, rinex_unix_t)
        if sat_ecef is None:
            continue

        # Elevation and azimuth
        elev, azim = elevation_azimuth(rx_ecef, sat_ecef)
        if elev < args.min_elev:
            continue

        # Best pseudorange (system-specific priority)
        psr_priority = PSR_KEYS.get(sys_char, ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'])
        psr = 0.0
        for key in psr_priority:
            if key in obs.pseudorange and obs.pseudorange[key] > 0:
                psr = obs.pseudorange[key]
                break
        if not psr and obs.pseudorange:
            psr = next((v for v in obs.pseudorange.values() if v > 0), 0.0)

        # Best CN0
        cn0 = 0.0
        for key in ['S1C', 'S1P', 'S1X', 'S2C', 'S1I', 'S1B']:
            if key in obs.cn0 and obs.cn0[key] > 0:
                cn0 = obs.cn0[key]
                break
        if not cn0 and obs.cn0:
            cn0 = next((v for v in obs.cn0.values() if v > 0), 0.0)

        rows.append({
            'unix_t':        f'{rinex_unix_t:.3f}',
            'utc_t':         f'{utc_t:.3f}',
            'sat_id':        sat_id,
            'sys':           sys_char,
            'prn':           obs.prn,
            'elevation_deg': f'{elev:.4f}',
            'azimuth_deg':   f'{azim:.4f}',
            'pseudorange':   f'{psr:.3f}',
            'cn0':           f'{cn0:.2f}',
            'rx_lat':        f'{rx_lat:.8f}',
            'rx_lon':        f'{rx_lon:.8f}',
            'rx_alt':        f'{rx_alt:.3f}',
        })
        sat_count += 1

    if (ep_i + 1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} 历元，已累积 {len(rows)} 条记录',
              end='', flush=True)

print(f'\n完成，共 {len(rows)} 条（卫星×历元）记录')
if no_ephem_sats:
    print(f'  无星历卫星：{sorted(no_ephem_sats)}')
if traj_misses:
    print(f'  超出 INSPVAX 时间范围历元数：{traj_misses}')

# ── write CSV ──────────────────────────────────────────────────────────────
COLS = ['unix_t', 'utc_t', 'sat_id', 'sys', 'prn',
        'elevation_deg', 'azimuth_deg', 'pseudorange', 'cn0',
        'rx_lat', 'rx_lon', 'rx_alt']

with open(args.out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS)
    w.writeheader()
    w.writerows(rows)

print(f'已保存：{args.out}')

# ── per-constellation statistics ───────────────────────────────────────────
if rows:
    elevs  = [float(r['elevation_deg']) for r in rows]
    azims  = [float(r['azimuth_deg'])   for r in rows]
    sats   = set(r['sat_id'] for r in rows)
    epochs_with_data = set(r['unix_t'] for r in rows)

    sys_cnt: dict = {}
    for r in rows:
        sys_cnt[r['sys']] = sys_cnt.get(r['sys'], 0) + 1

    print(f'\n--- 统计 ---')
    print(f'历元数（有数据）: {len(epochs_with_data)}')
    print(f'卫星种类:         {len(sats)} 颗')
    print(f'仰角范围:         {min(elevs):.1f}° ~ {max(elevs):.1f}°，均值 {np.mean(elevs):.1f}°')
    print(f'方位角范围:       {min(azims):.1f}° ~ {max(azims):.1f}°')
    print(f'\n星座分布（卫星×历元记录数）:')
    for sys_char in ('G', 'C', 'E'):
        cnt = sys_cnt.get(sys_char, 0)
        label = {'G': 'GPS', 'C': 'BeiDou', 'E': 'Galileo'}[sys_char]
        pct = 100.0 * cnt / len(rows) if rows else 0.0
        print(f'  {label:8s} ({sys_char}): {cnt:6d} 条  ({pct:.1f}%)')
    total_known = sum(sys_cnt.get(s, 0) for s in ('G', 'C', 'E'))
    other = len(rows) - total_known
    if other:
        print(f'  其他:           {other:6d} 条')
