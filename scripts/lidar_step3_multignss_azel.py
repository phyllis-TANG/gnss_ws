#!/usr/bin/env python3
"""
lidar_step3_multignss_azel.py
Multi-GNSS (GPS + BeiDou + Galileo) per-epoch azimuth/elevation calculation.

For each RINEX 3 obs epoch, computes elevation and azimuth for every
visible G/C/E satellite using the matching nav file for each constellation.
Receiver position is taken from NovAtel INSPVAX trajectory (novatel_trajectory.csv).

Output: epoch_sat_azel_multignss.csv  (same columns as step3, sys field captures constellation)

Usage (inside container):
  python3 lidar_step3_multignss_azel.py \\
    --obs     /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
    --nav_gps /root/urbannav_gnss/hksc137c.21n \\
    --nav_bds /root/urbannav_gnss/hksc137c.21f \\
    --nav_gal /root/urbannav_gnss/hksc137c.21l \\
    --traj    /root/novatel_trajectory.csv \\
    --out     /root/epoch_sat_azel_multignss.csv \\
    --min_elev 5.0

Time-handling note:
  read_rinex_obs treats GPS epochs as UTC, so epoch.time_unix is 18 s ahead of true UTC.
  compute_sat_position's unix_to_gpst has the same 18-s offset, so the two cancel out
  and satellite positions are correct.  For matching the INSPVAX trajectory (true UTC)
  we subtract LEAP_SECONDS=18 from epoch.time_unix.
"""

import argparse, csv, math, sys, os
import numpy as np

# ── rinex_utils search path (same pattern as existing step3/step7) ────────────
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

# Pseudorange observation code priority per constellation
PSR_KEYS = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
}

# ── CLI ───────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser(
    description='Multi-GNSS (G/C/E) azimuth & elevation computation')
ap.add_argument('--obs',     default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps', default='/root/urbannav_gnss/hksc137c.21n',
                help='GPS RINEX 2 nav file (.21n)')
ap.add_argument('--nav_bds', default='/root/urbannav_gnss/hksc137c.21f',
                help='BeiDou RINEX 2 nav file (.21f)')
ap.add_argument('--nav_gal', default='/root/urbannav_gnss/hksc137c.21l',
                help='Galileo RINEX 2 nav file (.21l)')
ap.add_argument('--traj',    default='/root/novatel_trajectory.csv')
ap.add_argument('--out',     default='/root/epoch_sat_azel_multignss.csv')
ap.add_argument('--min_elev', type=float, default=5.0,
                help='Elevation cut-off angle in degrees (default 5.0)')
args = ap.parse_args()


# ── Nav loading helper ────────────────────────────────────────────────────────
def load_nav(path, sys_prefix):
    """
    Load a RINEX nav file and remap all satellite keys to canonical format
    '<sys_prefix><PRN:02d>' (e.g. 'G01', 'C03', 'E11').

    Key remapping rules:
      - int key          → f'{sys_prefix}{key:02d}'
      - string with wrong leading letter → replace first char with sys_prefix
      - string already matching sys_prefix → keep as-is

    Returns a dict {sat_id: [ephem, ...]} or {} on failure.
    """
    if not os.path.isfile(path):
        print(f'  [warning] nav file not found, skipping: {path}')
        return {}
    try:
        raw = read_rinex_nav(path)
    except Exception as exc:
        print(f'  [warning] failed to read {path}: {exc}')
        return {}

    remapped = {}
    for key, val in raw.items():
        if isinstance(key, int):
            new_key = f'{sys_prefix}{key:02d}'
        elif isinstance(key, str):
            if len(key) > 1 and key[0].isalpha() and key[0] != sys_prefix:
                new_key = sys_prefix + key[1:]
            else:
                new_key = key
        else:
            new_key = str(key)
        remapped[new_key] = val

    return remapped


# ── Load trajectory ───────────────────────────────────────────────────────────
print('Loading INSPVAX trajectory...')
traj = []
with open(args.traj) as f:
    for row in csv.DictReader(f):
        traj.append((
            float(row['unix_t']),
            float(row['lat']), float(row['lon']), float(row['alt_m'])
        ))
traj.sort(key=lambda x: x[0])
traj_times = np.array([r[0] for r in traj])
print(f'  {len(traj)} poses, UTC {traj[0][0]:.1f} ~ {traj[-1][0]:.1f}')


def get_rx_ecef(utc_t):
    """Nearest-neighbour interpolation — returns (ecef, lat, lon, alt)."""
    idx = int(np.searchsorted(traj_times, utc_t))
    idx = min(max(idx, 0), len(traj) - 1)
    if idx > 0 and abs(traj_times[idx - 1] - utc_t) < abs(traj_times[idx] - utc_t):
        idx -= 1
    _, lat, lon, alt = traj[idx]
    return llh_to_ecef(lat, lon, alt), lat, lon, alt


# ── Load nav files ────────────────────────────────────────────────────────────
print('Loading navigation messages...')
nav_gps = load_nav(args.nav_gps, 'G')
nav_bds = load_nav(args.nav_bds, 'C')
nav_gal = load_nav(args.nav_gal, 'E')

ephem_dict = {}
ephem_dict.update(nav_gps)
ephem_dict.update(nav_bds)
ephem_dict.update(nav_gal)

total_ephem = sum(len(v) for v in ephem_dict.values())
print(f'  GPS:{len(nav_gps)} sats  BDS:{len(nav_bds)} sats  GAL:{len(nav_gal)} sats')
print(f'  Total: {len(ephem_dict)} satellites, {total_ephem} ephemeris records')

# ── Load observations ─────────────────────────────────────────────────────────
print('Loading observation file (may take a few seconds)...')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} epochs')

# ── Main loop ─────────────────────────────────────────────────────────────────
print(f'\nComputing azimuth/elevation (min_elev={args.min_elev}°)...')

rows = []
no_ephem_sats = set()
traj_misses   = 0
sys_counts    = {'G': 0, 'C': 0, 'E': 0}

for ep_i, epoch in enumerate(obs_epochs):
    rinex_unix_t = epoch.time_unix           # for sat position (offsets cancel)
    utc_t        = rinex_unix_t - LEAP_SECONDS  # for trajectory matching

    if utc_t < traj_times[0] - 2.0 or utc_t > traj_times[-1] + 2.0:
        traj_misses += 1
        continue

    rx_ecef, rx_lat, rx_lon, rx_alt = get_rx_ecef(utc_t)

    sat_count = 0
    for obs in epoch.obs_list:
        sat_id   = obs.sat_id
        sys_char = obs.sys

        # Only process GPS, BeiDou, Galileo
        if sys_char not in ('G', 'C', 'E'):
            continue

        eph_list = ephem_dict.get(sat_id, [])
        eph = find_closest_ephem(eph_list, rinex_unix_t)
        if eph is None:
            no_ephem_sats.add(sat_id)
            continue

        sat_ecef, dt_sv = compute_sat_position(eph, rinex_unix_t)
        if sat_ecef is None:
            continue

        elev, azim = elevation_azimuth(rx_ecef, sat_ecef)
        if elev < args.min_elev:
            continue

        # Best pseudorange (constellation-specific priority)
        psr = 0.0
        for key in PSR_KEYS.get(sys_char, ['C1C', 'C1P', 'C1X']):
            if key in obs.pseudorange and obs.pseudorange[key] > 0:
                psr = obs.pseudorange[key]
                break
        if not psr and obs.pseudorange:
            psr = next(iter(obs.pseudorange.values()))

        # Best CN0
        cn0 = 0.0
        for key in ['S1C', 'S1P', 'S1X', 'S2C', 'S1I', 'S1B']:
            if key in obs.cn0 and obs.cn0[key] > 0:
                cn0 = obs.cn0[key]
                break
        if not cn0 and obs.cn0:
            cn0 = next(iter(obs.cn0.values()))

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
        sys_counts[sys_char] = sys_counts.get(sys_char, 0) + 1
        sat_count += 1

    if (ep_i + 1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} epochs, {len(rows)} records accumulated',
              end='', flush=True)

print(f'\nDone: {len(rows)} (satellite×epoch) records')
if no_ephem_sats:
    print(f'  Satellites with no ephemeris: {sorted(no_ephem_sats)}')
if traj_misses:
    print(f'  Epochs outside INSPVAX time range: {traj_misses}')

# ── Write CSV ─────────────────────────────────────────────────────────────────
COLS = ['unix_t', 'utc_t', 'sat_id', 'sys', 'prn',
        'elevation_deg', 'azimuth_deg', 'pseudorange', 'cn0',
        'rx_lat', 'rx_lon', 'rx_alt']

with open(args.out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS)
    w.writeheader()
    w.writerows(rows)

print(f'Saved: {args.out}')

# ── Stats ─────────────────────────────────────────────────────────────────────
if rows:
    elevs  = [float(r['elevation_deg']) for r in rows]
    azims  = [float(r['azimuth_deg'])   for r in rows]
    sats   = set(r['sat_id'] for r in rows)
    epochs_with_data = set(r['unix_t'] for r in rows)

    print(f'\n--- Statistics ---')
    print(f'Epochs with data:  {len(epochs_with_data)}')
    print(f'Unique satellites: {len(sats)}')
    print(f'Elevation range:   {min(elevs):.1f}° ~ {max(elevs):.1f}°,  '
          f'mean {np.mean(elevs):.1f}°')
    print(f'Azimuth range:     {min(azims):.1f}° ~ {max(azims):.1f}°')
    print(f'\nPer-constellation record counts:')
    for sys_char in ('G', 'C', 'E'):
        label = {'G': 'GPS', 'C': 'BeiDou', 'E': 'Galileo'}[sys_char]
        cnt = sys_counts.get(sys_char, 0)
        pct = 100.0 * cnt / len(rows) if rows else 0.0
        print(f'  {label:7s} ({sys_char}): {cnt:6d}  ({pct:.1f}%)')
