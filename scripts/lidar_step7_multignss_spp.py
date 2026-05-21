#!/usr/bin/env python3
"""
lidar_step7_multignss_spp.py
Multi-GNSS (GPS / BeiDou / Galileo) version of lidar_step7_spp_correction.py.

LiDAR NLOS labels and delta-L estimates are used to correct pseudoranges
across all three constellations.  Three processing modes are compared:
  baseline   — all satellites, no corrections
  exclusion  — drop LiDAR-identified NLOS satellites
  correction — keep NLOS satellites, subtract delta-L, down-weight by planarity

The WLS solver uses independent clock biases per constellation
(up to 6 unknowns: X, Y, Z, c*dt_G, c*dt_C, c*dt_E).

Core references:
  [1] Kaplan & Hegarty (2006), Understanding GPS, Ch.7 — WLS SPP
  [2] Groves (2013), Principles of GNSS, Ch.9  — elevation weighting
  [3] Wen et al. (2019), NAVIGATION doi:10.1002/navi.335 — LiDAR NLOS correction
  [4] IS-GPS-200 (ICD) — satellite clock & Sagnac correction

Usage (in container):
  python3 lidar_step7_multignss_spp.py \
    --obs      /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
    --nav_gps  /root/urbannav_gnss/hksc137c.21n \
    --nav_bds  /root/urbannav_gnss/hksc137c.21f \
    --nav_gal  /root/urbannav_gnss/hksc137c.21l \
    --nlos     /root/lidar_nlos_prediction.csv \
    --refl     /root/lidar_reflection_model.csv \
    --gt       /root/urbannav_gt.txt \
    --out_csv  /root/spp_multignss_results.csv \
    --out_html /root/spp_multignss_report.html
"""

import argparse, csv, math, os, sys, json, datetime
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
    llh_to_ecef, ecef_to_llh,
)

LEAP_SECONDS = 18
C_LIGHT      = 299792458.0
OMEGA_E      = 7.2921151467e-5   # Earth rotation rate (rad/s)

# System-specific pseudorange priority keys
PSR_KEYS = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
}

# ── argument parsing ───────────────────────────────────────────────────────
ap = argparse.ArgumentParser(
    description='Multi-GNSS WLS SPP with LiDAR NLOS correction (G/C/E)')
ap.add_argument('--obs',     default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps', default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds', default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal', default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--nlos',    default='/root/lidar_nlos_prediction.csv')
ap.add_argument('--refl',    default='/root/lidar_reflection_model.csv')
ap.add_argument('--gt',      default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',  default='/root/spp_multignss_results.csv')
ap.add_argument('--out_html', default='/root/spp_multignss_report.html')
ap.add_argument('--min_elev', type=float, default=10.0,
                help='Elevation cutoff in degrees (default 10 deg)')
ap.add_argument('--gt_tol',   type=float, default=10.0,
                help='GT time matching tolerance in seconds (default 10 s)')
ap.add_argument('--nlos_weight_factor', type=float, default=0.1,
                help='Weight reduction factor for NLOS correction mode (default 0.1)')
args = ap.parse_args()


# ── nav loading helper ─────────────────────────────────────────────────────
def load_nav(path, sys_prefix):
    """
    Load a RINEX nav file and remap satellite keys to '<sys_prefix><nn>'.

    Handles three formats from read_rinex_nav:
      - integer  : PRN number -> '<prefix>{:02d}'
      - string with wrong leading letter -> replace first character
      - string already correct -> unchanged

    Returns an empty dict on any read failure (with a warning).
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
            new_key = key if key[0] == sys_prefix else sys_prefix + key[1:]
        else:
            new_key = key
        remapped[new_key] = ephem_list

    return remapped


# ── coordinate helpers ─────────────────────────────────────────────────────
def ecef_to_enu(ecef, ref_ecef, ref_lat_deg, ref_lon_deg):
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    R = np.array([
        [-math.sin(lon),                math.cos(lon),               0],
        [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
        [ math.cos(lat)*math.cos(lon),  math.cos(lat)*math.sin(lon), math.sin(lat)],
    ])
    return R @ (ecef - ref_ecef)


def sagnac_correct(sat_ecef, travel_time_s):
    """Apply Sagnac (Earth-rotation) correction to satellite ECEF. [4]"""
    theta = OMEGA_E * travel_time_s
    c, s  = math.cos(theta), math.sin(theta)
    R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return R @ sat_ecef


def tropo_delay(elev_deg):
    """Simplified Hopfield tropospheric delay (m). Zenith ~ 2.3 m."""
    el = max(elev_deg, 3.0)
    return 2.3 / math.sin(math.radians(el) + 0.017)


# ── multi-GNSS WLS solver ──────────────────────────────────────────────────
def wls_spp_multi(sats_info, x0_ecef, min_sats=4, max_iter=10):
    """
    Iterative WLS SPP with independent per-constellation clock biases. [1][2]

    sats_info : list of dict, each with:
        sat_ecef  — satellite ECEF (Sagnac-corrected)
        psr_corr  — pseudorange after satellite-clock and troposphere correction
        weight    — observation weight (0 = excluded)
        elevation — elevation angle in degrees (for PDOP reference)
        sys       — constellation char: 'G', 'C', or 'E'

    Unknowns: [X, Y, Z, c*dt_G, c*dt_C, c*dt_E]
      Only clock columns for constellations present among active satellites
      are included, so the actual state vector length is 3 + n_constellations.

    Minimum required observations: 3 + n_constellations.

    Returns (pos_ecef[3], clk_dict, pdop) or (None, None, None).
    clk_dict maps constellation char -> clock bias in metres.
    PDOP is computed from the 3x3 position submatrix of (H^T H)^{-1}.
    """
    active = [s for s in sats_info if s['weight'] > 0]
    if not active:
        return None, None, None

    # Determine which constellations are present
    sys_present = sorted(set(s['sys'] for s in active))
    n_clk = len(sys_present)
    min_required = 3 + n_clk
    if len(active) < max(min_sats, min_required):
        return None, None, None

    # Map constellation char -> column index (offset by 3)
    clk_col = {sys: 3 + i for i, sys in enumerate(sys_present)}

    n_unknowns = 3 + n_clk
    # State vector: [X, Y, Z, c*dt_sys0, c*dt_sys1, ...]
    x = np.zeros(n_unknowns, dtype=float)
    x[:3] = x0_ecef

    for _ in range(max_iter):
        H_list, dp_list, w_list = [], [], []
        for s in active:
            diff = x[:3] - np.array(s['sat_ecef'])
            r    = np.linalg.norm(diff)
            if r < 1e4:
                continue
            e       = diff / r          # unit vector receiver -> satellite
            row     = np.zeros(n_unknowns)
            row[:3] = e
            row[clk_col[s['sys']]] = 1.0
            H_list.append(row)
            dp_list.append(s['psr_corr'] - r - x[clk_col[s['sys']]])
            w_list.append(s['weight'])

        if len(H_list) < max(min_sats, min_required):
            return None, None, None

        H  = np.array(H_list)
        dp = np.array(dp_list)
        W  = np.diag(w_list)

        HtW = H.T @ W
        try:
            delta = np.linalg.solve(HtW @ H, HtW @ dp)
        except np.linalg.LinAlgError:
            return None, None, None

        x += delta
        if np.linalg.norm(delta[:3]) < 0.01:
            break

    # PDOP: trace of 3x3 position submatrix of (H^T H)^{-1}
    try:
        Q    = np.linalg.inv(H.T @ H)
        pdop = math.sqrt(max(Q[0, 0] + Q[1, 1] + Q[2, 2], 0.0))
    except Exception:
        pdop = float('nan')

    clk_dict = {sys: float(x[col]) for sys, col in clk_col.items()}
    return x[:3], clk_dict, pdop


# ── load NLOS labels ───────────────────────────────────────────────────────
print('加载 LiDAR NLOS 标签...')
nlos_dict = {}   # (unix_t_str, sat_id) -> 1/0
with open(args.nlos) as f:
    for row in csv.DictReader(f):
        nlos_dict[(row['unix_t'], row['sat_id'])] = int(row['lidar_nlos'])
print(f'  {len(nlos_dict)} 条')

# ── load delta-L and planarity ─────────────────────────────────────────────
print('加载反射面 delta-L...')
refl_dict = {}   # (unix_t_str, sat_id) -> (delta_l, planarity)
with open(args.refl) as f:
    for row in csv.DictReader(f):
        dl   = float(row['delta_L_m'])  if row['delta_L_m']  else 0.0
        plan = float(row['planarity'])  if row['planarity']  else 0.3
        refl_dict[(row['unix_t'], row['sat_id'])] = (dl, plan)
print(f'  {len(refl_dict)} 条')

# ── load ground truth ──────────────────────────────────────────────────────
# UrbanNav GT format: space-separated, two header lines.
# Columns: UTCTime(unix s) Week GPSTime D M S_lat D M S_lon H-Ell ...
print(f'加载地面真值: {args.gt}')
gt_times, gt_lats, gt_lons, gt_alts = [], [], [], []

def dms_to_deg(d, m, s):
    return float(d) + float(m) / 60.0 + float(s) / 3600.0

with open(args.gt) as f:
    lines = f.readlines()

for line in lines:
    line = line.strip()
    if not line or line.startswith('UTC') or line.startswith('('):
        continue
    parts = line.split()
    if len(parts) < 10:
        continue
    try:
        utc_t = float(parts[0])
        lat   = dms_to_deg(parts[3], parts[4], parts[5])
        lon   = dms_to_deg(parts[6], parts[7], parts[8])
        alt   = float(parts[9])
        if utc_t < 1e9:
            continue
        gt_times.append(utc_t)
        gt_lats.append(lat)
        gt_lons.append(lon)
        gt_alts.append(alt)
    except (ValueError, IndexError):
        continue

gt_times = np.array(gt_times)
gt_lats  = np.array(gt_lats)
gt_lons  = np.array(gt_lons)
gt_alts  = np.array(gt_alts)
order = np.argsort(gt_times)
gt_times, gt_lats, gt_lons, gt_alts = (
    gt_times[order], gt_lats[order], gt_lons[order], gt_alts[order])
print(f'  {len(gt_times)} 个 GT 点，UTC {gt_times[0]:.1f} ~ {gt_times[-1]:.1f}')


def match_gt(utc_t):
    """Nearest-neighbour GT match; returns None if outside tolerance."""
    idx = int(np.searchsorted(gt_times, utc_t))
    idx = min(max(idx, 0), len(gt_times) - 1)
    if idx > 0 and abs(gt_times[idx - 1] - utc_t) < abs(gt_times[idx] - utc_t):
        idx -= 1
    if abs(gt_times[idx] - utc_t) > args.gt_tol:
        return None
    return gt_lats[idx], gt_lons[idx], gt_alts[idx]


# ── load nav files ─────────────────────────────────────────────────────────
print('读取导航电文（GPS / BeiDou / Galileo）...')
ephem_gps = load_nav(args.nav_gps, 'G')
ephem_bds = load_nav(args.nav_bds, 'C')
ephem_gal = load_nav(args.nav_gal, 'E')

ephem_dict = {}
ephem_dict.update(ephem_gps)
ephem_dict.update(ephem_bds)
ephem_dict.update(ephem_gal)

print(f'  GPS {len(ephem_gps)} 颗，BDS {len(ephem_bds)} 颗，GAL {len(ephem_gal)} 颗')

# ── load observations ──────────────────────────────────────────────────────
print('读取观测文件...')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} 个历元')

# Initial linearisation point (Hong Kong UrbanNav approximate)
x0_ecef = np.array(llh_to_ecef(22.3198, 114.2095, 20.0))

# ── main loop ──────────────────────────────────────────────────────────────
print(f'\n运行 Multi-GNSS SPP（三模式，min_elev={args.min_elev}°）...')

results = []
no_gt   = 0

for ep_i, epoch in enumerate(obs_epochs):
    rinex_unix_t = epoch.time_unix
    utc_t        = rinex_unix_t - LEAP_SECONDS

    gt_match = match_gt(utc_t)
    if gt_match is None:
        no_gt += 1
        continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    # ── build per-satellite observation table ──────────────────────────────
    sat_data = []

    for obs in epoch.obs_list:
        sat_id   = obs.sat_id
        sys_char = obs.sys if obs.sys else sat_id[0]

        # Only process GPS, BeiDou and Galileo
        if sys_char not in ('G', 'C', 'E'):
            continue

        eph_list = ephem_dict.get(sat_id, [])
        eph = find_closest_ephem(eph_list, rinex_unix_t)
        if eph is None:
            continue

        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_unix_t)
        if sat_ecef_raw is None:
            continue

        # System-specific pseudorange selection
        psr_raw = 0.0
        for key in PSR_KEYS.get(sys_char, ['C1C', 'C1P', 'C1X', 'C2C', 'C2P']):
            if key in obs.pseudorange and obs.pseudorange[key] > 0:
                psr_raw = obs.pseudorange[key]
                break
        if psr_raw <= 0:
            continue

        # Satellite clock correction [4]
        psr_clk = psr_raw + C_LIGHT * dt_sv

        # Sagnac correction [4]
        travel_t   = psr_raw / C_LIGHT
        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), travel_t)

        # Approximate elevation using x0_ecef as linearisation point
        diff_rx  = np.array(sat_ecef_s) - x0_ecef
        r_approx = np.linalg.norm(diff_rx)
        gt_lat_r  = math.radians(gt_lat)
        gt_lon_r  = math.radians(gt_lon)
        up = np.array([
            math.cos(gt_lat_r) * math.cos(gt_lon_r),
            math.cos(gt_lat_r) * math.sin(gt_lon_r),
            math.sin(gt_lat_r),
        ])
        elev_approx = math.degrees(math.asin(
            np.clip(np.dot(diff_rx / r_approx, up), -1.0, 1.0)))

        if elev_approx < args.min_elev:
            continue

        # Tropospheric correction
        psr_tropo = psr_clk - tropo_delay(elev_approx)

        # Elevation-based weight [2]
        w_base = math.sin(math.radians(elev_approx)) ** 2

        # NLOS label and delta-L (default to LOS if not in tables)
        key_nt  = (f'{rinex_unix_t:.3f}', sat_id)
        is_nlos = nlos_dict.get(key_nt, 0)
        delta_l, planarity = refl_dict.get(key_nt, (0.0, 0.5))

        sat_data.append({
            'sat_id':    sat_id,
            'sys':       sys_char,
            'sat_ecef':  sat_ecef_s,
            'psr_tropo': psr_tropo,
            'elev':      elev_approx,
            'w_base':    w_base,
            'is_nlos':   is_nlos,
            'delta_l':   delta_l,
            'planarity': planarity,
        })

    if len(sat_data) < 4:
        continue

    n_total = len(sat_data)
    n_nlos  = sum(1 for s in sat_data if s['is_nlos'])
    n_gps   = sum(1 for s in sat_data if s['sys'] == 'G')
    n_bds   = sum(1 for s in sat_data if s['sys'] == 'C')
    n_gal   = sum(1 for s in sat_data if s['sys'] == 'E')

    # ── three modes ───────────────────────────────────────────────────────
    mode_results = {}

    for mode in ('baseline', 'exclusion', 'correction'):
        sats_info = []
        for s in sat_data:
            if mode == 'baseline':
                w   = s['w_base']
                psr = s['psr_tropo']
            elif mode == 'exclusion':
                w   = 0.0 if s['is_nlos'] else s['w_base']
                psr = s['psr_tropo']
            else:   # correction
                if s['is_nlos']:
                    plan = s['planarity']
                    if plan >= 0.7:
                        # Reliable surface: apply correction, keep good weight
                        psr = s['psr_tropo'] - s['delta_l']
                        w   = s['w_base'] * plan * 0.5
                    elif plan >= 0.5:
                        # Medium quality: apply scaled correction, reduced weight
                        psr = s['psr_tropo'] - s['delta_l'] * plan
                        w   = s['w_base'] * 0.15
                    else:
                        # Low quality normal: no correction, just down-weight
                        psr = s['psr_tropo']
                        w   = s['w_base'] * 0.05
                else:
                    w   = s['w_base']
                    psr = s['psr_tropo']

            sats_info.append({
                'sat_ecef':  s['sat_ecef'],
                'psr_corr':  psr,
                'weight':    w,
                'elevation': s['elev'],
                'sys':       s['sys'],
            })

        pos, clk_dict, pdop = wls_spp_multi(sats_info, x0_ecef)

        if pos is None:
            mode_results[mode] = None
            continue

        err_enu  = ecef_to_enu(pos, gt_ecef, gt_lat, gt_lon)
        err_h    = math.sqrt(err_enu[0] ** 2 + err_enu[1] ** 2)
        err_v    = abs(err_enu[2])
        err_3d   = math.sqrt(err_h ** 2 + err_v ** 2)

        mode_results[mode] = {
            'err_h':  round(err_h,  2),
            'err_v':  round(err_v,  2),
            'err_3d': round(err_3d, 2),
            'pdop':   round(pdop,   2) if not math.isnan(pdop) else None,
            'n_used': sum(1 for si in sats_info if si['weight'] > 0),
        }

    if all(v is None for v in mode_results.values()):
        continue

    results.append({
        'utc_t':   round(utc_t, 3),
        'n_total': n_total,
        'n_nlos':  n_nlos,
        'n_gps':   n_gps,
        'n_bds':   n_bds,
        'n_gal':   n_gal,
        **{f'{m}_{k}': (mode_results[m][k] if mode_results[m] else None)
           for m in ('baseline', 'exclusion', 'correction')
           for k in ('err_h', 'err_v', 'err_3d', 'pdop', 'n_used')},
    })

    if (ep_i + 1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} 历元...', end='', flush=True)

print(f'\n完成：{len(results)} 个有效历元（{no_gt} 个超出 GT 时间范围）')

# ── write CSV ──────────────────────────────────────────────────────────────
COLS = [
    'utc_t', 'n_total', 'n_nlos', 'n_gps', 'n_bds', 'n_gal',
    'baseline_err_h',   'baseline_err_v',   'baseline_err_3d',
    'baseline_pdop',    'baseline_n_used',
    'exclusion_err_h',  'exclusion_err_v',  'exclusion_err_3d',
    'exclusion_pdop',   'exclusion_n_used',
    'correction_err_h', 'correction_err_v', 'correction_err_3d',
    'correction_pdop',  'correction_n_used',
]
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS, extrasaction='ignore')
    w.writeheader()
    w.writerows(results)
print(f'已保存 CSV：{args.out_csv}')

# ── statistics helpers ─────────────────────────────────────────────────────
def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return dict(n=0, mean=float('nan'), rms=float('nan'),
                    p50=float('nan'), p95=float('nan'))
    a = np.array(v)
    return dict(
        n    = len(v),
        mean = float(np.mean(a)),
        rms  = float(np.sqrt(np.mean(a ** 2))),
        p50  = float(np.percentile(a, 50)),
        p95  = float(np.percentile(a, 95)),
    )

modes = ('baseline', 'exclusion', 'correction')
labels_cn = {
    'baseline':   '基准（无修正）',
    'exclusion':  'NLOS 排除',
    'correction': 'NLOS 修正（delta-L）',
}

print('\n--- 定位精度统计（水平误差）---')
for mode in modes:
    s = stats([r[f'{mode}_err_h'] for r in results])
    print(f'  {mode:12s}  n={s["n"]:4d}  '
          f'均值={s["mean"]:7.2f}m  RMS={s["rms"]:7.2f}m  '
          f'50th={s["p50"]:7.2f}m  95th={s["p95"]:7.2f}m')

print('\n--- 卫星数统计（平均每历元）---')
if results:
    avg_gps = np.mean([r['n_gps']   for r in results])
    avg_bds = np.mean([r['n_bds']   for r in results])
    avg_gal = np.mean([r['n_gal']   for r in results])
    avg_tot = np.mean([r['n_total'] for r in results])
    avg_nls = np.mean([r['n_nlos']  for r in results])
    print(f'  GPS:     {avg_gps:.1f} 颗/历元')
    print(f'  BeiDou:  {avg_bds:.1f} 颗/历元')
    print(f'  Galileo: {avg_gal:.1f} 颗/历元')
    print(f'  总计:    {avg_tot:.1f} 颗/历元（其中 NLOS {avg_nls:.1f} 颗）')

# ── HTML report ────────────────────────────────────────────────────────────
def cdf_data(vals, n_pts=200):
    v = sorted(x for x in vals if x is not None)
    if not v:
        return []
    mn = v[0]
    mx = min(v[-1], float(np.percentile(v, 99)) * 1.2)
    xs = np.linspace(mn, mx, n_pts)
    ys = [sum(1 for vi in v if vi <= x) / len(v) for x in xs]
    return [{'x': round(float(x), 2), 'y': round(y, 4)} for x, y in zip(xs, ys)]

def ts_data(results, key):
    return [{'x': r['utc_t'], 'y': r[key]} for r in results if r[key] is not None]

colors = {
    'baseline':   'rgba(244,67,54,.7)',
    'exclusion':  'rgba(255,152,0,.8)',
    'correction': 'rgba(33,150,243,.85)',
}
sys_colors = {
    'G': 'rgba(33,150,243,.8)',
    'C': 'rgba(76,175,80,.8)',
    'E': 'rgba(255,152,0,.8)',
}

st = {m: stats([r[f'{m}_err_h'] for r in results]) for m in modes}

cdf_js   = {m: json.dumps(cdf_data([r[f'{m}_err_h'] for r in results]))  for m in modes}
ts_js    = {m: json.dumps(ts_data(results, f'{m}_err_h'))                 for m in modes}
ts_3d_js = {m: json.dumps(ts_data(results, f'{m}_err_3d'))                for m in modes}

bar_labels = json.dumps(['均值 (m)', 'RMS (m)', '50th (m)', '95th (m)'])
bar_data   = {m: json.dumps([round(st[m][k], 2)
                              for k in ('mean', 'rms', 'p50', 'p95')])
              for m in modes}

nlos_ts_js  = json.dumps([{'x': r['utc_t'], 'y': r['n_nlos']}  for r in results])
total_ts_js = json.dumps([{'x': r['utc_t'], 'y': r['n_total']} for r in results])
gps_ts_js   = json.dumps([{'x': r['utc_t'], 'y': r['n_gps']}   for r in results])
bds_ts_js   = json.dumps([{'x': r['utc_t'], 'y': r['n_bds']}   for r in results])
gal_ts_js   = json.dumps([{'x': r['utc_t'], 'y': r['n_gal']}   for r in results])

avg_gps_val = float(np.mean([r['n_gps']   for r in results])) if results else 0.0
avg_bds_val = float(np.mean([r['n_bds']   for r in results])) if results else 0.0
avg_gal_val = float(np.mean([r['n_gal']   for r in results])) if results else 0.0
avg_tot_val = float(np.mean([r['n_total'] for r in results])) if results else 0.0

html = f"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<title>Step 7 Multi-GNSS — SPP NLOS Correction Report</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  body {{ font-family: 'Segoe UI',Arial,sans-serif; margin:20px; background:#f4f6fb; color:#333; }}
  h1   {{ color:#1a237e; border-bottom:3px solid #3f51b5; padding-bottom:8px; }}
  h2   {{ color:#283593; margin-top:28px; }}
  h3   {{ color:#37474f; margin:0 0 6px; }}
  .section {{ background:white; padding:20px; margin:14px 0;
              border-radius:8px; box-shadow:0 1px 5px rgba(0,0,0,.1); }}
  .grid2   {{ display:grid; grid-template-columns:1fr 1fr; gap:20px; }}
  .grid3   {{ display:grid; grid-template-columns:1fr 1fr 1fr; gap:16px; }}
  .grid4   {{ display:grid; grid-template-columns:1fr 1fr 1fr 1fr; gap:14px; }}
  .card    {{ padding:16px; border-radius:8px; text-align:center; }}
  .card .val {{ font-size:1.8em; font-weight:bold; }}
  .card .lbl {{ font-size:.82em; color:#555; margin-top:4px; }}
  .base {{ background:#ffebee; }} .excl {{ background:#fff3e0; }}
  .corr {{ background:#e3f2fd; }} .sat  {{ background:#f1f8e9; }}
  .delta {{ font-size:.9em; font-weight:bold; color:#388e3c; }}
  table {{ border-collapse:collapse; width:100%; font-size:.9em; }}
  th {{ background:#3f51b5; color:#fff; padding:8px 12px; text-align:left; }}
  td {{ padding:7px 12px; border-bottom:1px solid #e8e8e8; }}
  tr:nth-child(even) {{ background:#f5f7ff; }}
  .better {{ color:#2e7d32; font-weight:bold; }} .worse {{ color:#c62828; }}
  .ref {{ font-size:.8em; color:#666; line-height:1.9; }}
  .chart-box {{ padding:8px; }}
  .note {{ font-size:.82em; color:#777; margin:3px 0 10px; }}
  .formula {{ background:#f8f9fa; border-left:4px solid #3f51b5;
              padding:8px 14px; font-family:monospace; margin:8px 0; font-size:.92em; }}
</style>
</head>
<body>
<h1>Step 7 Multi-GNSS &mdash; SPP NLOS Correction Report</h1>
<p>UrbanNav Medium-Urban-1（香港 TST，2021-05-17）&nbsp;&middot;&nbsp;
   Constellations: GPS + BeiDou + Galileo &nbsp;&middot;&nbsp;
   Valid epochs: {len(results)} &nbsp;&middot;&nbsp; Elevation cutoff: {args.min_elev}&deg;</p>

<div class="section">
  <h2>Method</h2>
  <div class="formula">
    Unknowns: [X, Y, Z, c&middot;&delta;t<sub>G</sub>, c&middot;&delta;t<sub>C</sub>, c&middot;&delta;t<sub>E</sub>]&nbsp;
    (only columns for present constellations are included)<br>
    H row GPS:     [e<sub>x</sub>, e<sub>y</sub>, e<sub>z</sub>,  1,  0,  0]<br>
    H row BeiDou:  [e<sub>x</sub>, e<sub>y</sub>, e<sub>z</sub>,  0,  1,  0]<br>
    H row Galileo: [e<sub>x</sub>, e<sub>y</sub>, e<sub>z</sub>,  0,  0,  1]<br><br>
    Baseline:   &rho;&#771;<sub>i</sub> = &rho;<sub>i</sub> &minus; c&middot;&delta;t<sub>sv</sub> &minus; T<sub>i</sub> (all sats, elev-weighted)<br>
    Exclusion:  drop LiDAR-NLOS satellites<br>
    Correction: &rho;&#771;<sub>corr</sub> = &rho;&#771;<sub>i</sub> &minus; &Delta;L<sub>i</sub>, weight &times; planarity &times; {args.nlos_weight_factor}
  </div>
  <p class="ref">
    [1] Kaplan &amp; Hegarty (2006) &mdash; Iterative WLS SPP<br>
    [2] Groves (2013) &mdash; Elevation weighting w = sin&sup2;(el)<br>
    [3] Wen et al. (2019) NAVIGATION doi:10.1002/navi.335 &mdash; LiDAR NLOS pseudorange correction<br>
    [4] IS-GPS-200 &mdash; Satellite clock &amp; Sagnac correction
  </p>
</div>

<div class="section">
  <h2>Satellite Availability Summary</h2>
  <div class="grid4">
    <div class="card sat">
      <div class="val">{avg_tot_val:.1f}</div>
      <div class="lbl">Avg total sats/epoch</div>
    </div>
    <div class="card" style="background:#e3f2fd">
      <div class="val">{avg_gps_val:.1f}</div>
      <div class="lbl">Avg GPS sats/epoch</div>
    </div>
    <div class="card" style="background:#e8f5e9">
      <div class="val">{avg_bds_val:.1f}</div>
      <div class="lbl">Avg BeiDou sats/epoch</div>
    </div>
    <div class="card" style="background:#fff3e0">
      <div class="val">{avg_gal_val:.1f}</div>
      <div class="lbl">Avg Galileo sats/epoch</div>
    </div>
  </div>
</div>

<div class="section">
  <h2>Core Results (Horizontal Error)</h2>
  <div class="grid3">
    <div class="card base">
      <div class="val">{st['baseline']['mean']:.1f} m</div>
      <div class="lbl">Baseline mean</div>
      <div class="lbl">RMS {st['baseline']['rms']:.1f}m &nbsp; 95th {st['baseline']['p95']:.1f}m</div>
    </div>
    <div class="card excl">
      <div class="val">{st['exclusion']['mean']:.1f} m</div>
      <div class="lbl">NLOS Exclusion mean</div>
      <div class="lbl">RMS {st['exclusion']['rms']:.1f}m &nbsp; 95th {st['exclusion']['p95']:.1f}m</div>
      <div class="delta">
        {"&darr;{:.1f}m".format(st['baseline']['mean']-st['exclusion']['mean'])
         if st['exclusion']['mean'] < st['baseline']['mean']
         else "&uarr;{:.1f}m".format(st['exclusion']['mean']-st['baseline']['mean'])}
      </div>
    </div>
    <div class="card corr">
      <div class="val">{st['correction']['mean']:.1f} m</div>
      <div class="lbl">NLOS Correction mean</div>
      <div class="lbl">RMS {st['correction']['rms']:.1f}m &nbsp; 95th {st['correction']['p95']:.1f}m</div>
      <div class="delta">
        {"&darr;{:.1f}m".format(st['baseline']['mean']-st['correction']['mean'])
         if st['correction']['mean'] < st['baseline']['mean']
         else "&uarr;{:.1f}m".format(st['correction']['mean']-st['baseline']['mean'])}
      </div>
    </div>
  </div>

  <h3 style="margin-top:20px">Detailed statistics</h3>
  <table>
    <tr><th>Mode</th><th>Valid epochs</th><th>Mean (m)</th><th>RMS (m)</th>
        <th>50th (m)</th><th>95th (m)</th><th>vs Baseline</th></tr>
    {''.join(
      f'<tr><td>{labels_cn[m]}</td>'
      f'<td>{st[m]["n"]}</td>'
      f'<td>{st[m]["mean"]:.2f}</td>'
      f'<td>{st[m]["rms"]:.2f}</td>'
      f'<td>{st[m]["p50"]:.2f}</td>'
      f'<td>{st[m]["p95"]:.2f}</td>'
      f'<td class="{"better" if st[m]["mean"]<=st["baseline"]["mean"] else "worse"}">'
      f'{"&minus;{:.2f}m".format(st["baseline"]["mean"]-st[m]["mean"]) if m!="baseline" else "&mdash;"}'
      f'</td></tr>'
      for m in modes
    )}
  </table>
</div>

<div class="section">
  <h2>Charts</h2>
  <div class="grid2">
    <div class="chart-box">
      <h3>Horizontal Error CDF</h3>
      <p class="note">Further left = better. Higher CDF at same error = more epochs within that bound.</p>
      <canvas id="cCDF" height="260"></canvas>
    </div>
    <div class="chart-box">
      <h3>Error Statistics Bar Chart</h3>
      <canvas id="cBar" height="260"></canvas>
    </div>
  </div>
  <div class="chart-box" style="margin-top:16px">
    <h3>Horizontal Error Time Series</h3>
    <p class="note">Per-epoch horizontal error for all three modes</p>
    <canvas id="cTS" height="200"></canvas>
  </div>
  <div class="chart-box" style="margin-top:16px">
    <h3>Satellite Count per Epoch (GPS / BeiDou / Galileo)</h3>
    <p class="note">Line chart showing per-constellation visible satellite counts</p>
    <canvas id="cSatConstel" height="180"></canvas>
  </div>
  <div class="chart-box" style="margin-top:16px">
    <h3>Total and NLOS Satellite Count per Epoch</h3>
    <canvas id="cSatNlos" height="160"></canvas>
  </div>
</div>

<script>
const MODES  = ['baseline','exclusion','correction'];
const COLORS = {{'baseline':'{colors["baseline"]}','exclusion':'{colors["exclusion"]}','correction':'{colors["correction"]}'}};
const LABELS = {{'baseline':'Baseline','exclusion':'NLOS Exclusion','correction':'NLOS Correction'}};

const cdfData = {{
  baseline:   {cdf_js['baseline']},
  exclusion:  {cdf_js['exclusion']},
  correction: {cdf_js['correction']}
}};
const tsData = {{
  baseline:   {ts_js['baseline']},
  exclusion:  {ts_js['exclusion']},
  correction: {ts_js['correction']}
}};
const barLabels = {bar_labels};
const barData   = {{
  baseline:   {bar_data['baseline']},
  exclusion:  {bar_data['exclusion']},
  correction: {bar_data['correction']}
}};
const nlosTs    = {nlos_ts_js};
const totalTs   = {total_ts_js};
const gpsTs     = {gps_ts_js};
const bdsTs     = {bds_ts_js};
const galTs     = {gal_ts_js};

// CDF chart
new Chart(document.getElementById('cCDF'), {{
  type: 'line',
  data: {{
    datasets: MODES.map(m => ({{
      label: LABELS[m],
      data:  cdfData[m],
      borderColor: COLORS[m].replace('.7','.9').replace('.8','.9').replace('.85','1'),
      backgroundColor: 'transparent',
      borderWidth: 2.5,
      pointRadius: 0,
    }}))
  }},
  options: {{
    parsing: {{xAxisKey:'x', yAxisKey:'y'}},
    plugins: {{legend: {{position:'top'}}}},
    scales: {{
      x: {{title: {{display:true, text:'Horizontal error (m)'}}}},
      y: {{title: {{display:true, text:'CDF'}}, min:0, max:1}}
    }}
  }}
}});

// Bar chart
new Chart(document.getElementById('cBar'), {{
  type: 'bar',
  data: {{
    labels: barLabels,
    datasets: MODES.map(m => ({{
      label: LABELS[m],
      data: barData[m],
      backgroundColor: COLORS[m]
    }}))
  }},
  options: {{
    plugins: {{legend: {{position:'top'}}}},
    scales: {{y: {{title: {{display:true, text:'Error (m)'}}}}}}
  }}
}});

// Time series
new Chart(document.getElementById('cTS'), {{
  type: 'line',
  data: {{
    datasets: MODES.map(m => ({{
      label: LABELS[m],
      data: tsData[m],
      borderColor: COLORS[m].replace('.7','1').replace('.8','1').replace('.85','1'),
      backgroundColor: COLORS[m].replace('.7','.15').replace('.8','.15').replace('.85','.15'),
      borderWidth: 1.2,
      pointRadius: 0,
      fill: false,
    }}))
  }},
  options: {{
    parsing: {{xAxisKey:'x', yAxisKey:'y'}},
    plugins: {{legend: {{position:'top'}}}},
    scales: {{
      x: {{type:'linear', title: {{display:true, text:'UTC timestamp (s)'}}}},
      y: {{title: {{display:true, text:'Horizontal error (m)'}}, min:0}}
    }}
  }}
}});

// Per-constellation satellite count
new Chart(document.getElementById('cSatConstel'), {{
  type: 'line',
  data: {{
    datasets: [
      {{label:'GPS',     data:gpsTs, borderColor:'{sys_colors["G"]}',
        borderWidth:1.8, pointRadius:0, fill:false}},
      {{label:'BeiDou',  data:bdsTs, borderColor:'{sys_colors["C"]}',
        borderWidth:1.8, pointRadius:0, fill:false}},
      {{label:'Galileo', data:galTs, borderColor:'{sys_colors["E"]}',
        borderWidth:1.8, pointRadius:0, fill:false}},
    ]
  }},
  options: {{
    parsing: {{xAxisKey:'x', yAxisKey:'y'}},
    plugins: {{legend: {{position:'top'}}}},
    scales: {{
      x: {{type:'linear', title: {{display:true, text:'UTC timestamp (s)'}}}},
      y: {{title: {{display:true, text:'Satellite count'}}, min:0}}
    }}
  }}
}});

// Total vs NLOS satellite count
new Chart(document.getElementById('cSatNlos'), {{
  type: 'line',
  data: {{
    datasets: [
      {{label:'Total sats', data:totalTs, borderColor:'rgba(100,100,200,.8)',
        borderWidth:1.5, pointRadius:0, fill:false}},
      {{label:'NLOS sats',  data:nlosTs,  borderColor:'rgba(244,67,54,.7)',
        borderWidth:1.5, pointRadius:0, fill:true,
        backgroundColor:'rgba(244,67,54,.1)'}}
    ]
  }},
  options: {{
    parsing: {{xAxisKey:'x', yAxisKey:'y'}},
    plugins: {{legend: {{position:'top'}}}},
    scales: {{
      x: {{type:'linear', title: {{display:true, text:'UTC timestamp (s)'}}}},
      y: {{title: {{display:true, text:'Satellite count'}}, min:0}}
    }}
  }}
}});
</script>
</body>
</html>"""

with open(args.out_html, 'w', encoding='utf-8') as f:
    f.write(html)
print(f'已保存 HTML：{args.out_html}')
