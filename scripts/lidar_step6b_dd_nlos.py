#!/usr/bin/env python3
"""
lidar_step6b_dd_nlos.py
Double-difference (DD) NLOS pre-screening using HKSC reference station.

Method:
  For each rover epoch matched to GT:
    1. Find the same-time HKSC reference epoch (±1 s)
    2. For each GPS satellite visible to both rover and reference:
         SD_geometric  = |rover_pos → sat| − |ref_pos → sat|
         SD_measured   = rover_psr − ref_psr
         SD_residual   = SD_measured − SD_geometric
       (receiver clock difference is common to all SD_residuals)
    3. Pick pivot satellite = highest-elevation GPS sat
         DD_residual[i] = SD_residual[i] − SD_residual[pivot]
       (clock difference cancels; pivot assumed LOS → nlos[pivot]≈0)
    4. |DD_residual| > threshold  →  DD-flagged NLOS

  Then compare with LiDAR 2-bounce labels and report confusion matrix.

Usage (inside container):
  python3 lidar_step6b_dd_nlos.py \\
    --obs     /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
    --nav_gps /root/urbannav_gnss/hksc137c.21n \\
    --ref_obs /root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/data/UrbanNavMedium/base/hksc137c.21o \\
    --nlos_2b /root/lidar_nlos_prediction_2b.csv \\
    --gt      /root/urbannav_gt.txt \\
    --out_csv  /root/dd_nlos_labels.csv \\
    --out_html /root/dd_nlos_report.html
"""

import argparse, base64, csv, datetime, io, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict

# ── rinex_utils path ─────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in [
    '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts',
    '/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts',
    os.path.join(SCRIPT_DIR, '../src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts'),
]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rinex_utils import (read_rinex_obs, read_rinex_nav,
                          compute_sat_position, find_closest_ephem, llh_to_ecef)

# ── constants ────────────────────────────────────────────────────────
LEAP_SECONDS   = 18
C_LIGHT        = 299792458.0
OMEGA_E        = 7.2921151467e-5
GPS_EPOCH_UNIX = 315964800   # Unix seconds at GPS epoch 1980-01-06 00:00:00 UTC

ap = argparse.ArgumentParser()
ap.add_argument('--obs',       default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps',   default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--ref_obs',   default='/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/data/UrbanNavMedium/base/hksc137c.21o')
ap.add_argument('--nlos_2b',   default='/root/lidar_nlos_prediction_2b.csv')
ap.add_argument('--gt',        default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',   default='/root/dd_nlos_labels.csv')
ap.add_argument('--out_html',  default='/root/dd_nlos_report.html')
ap.add_argument('--dd_thresh', type=float, default=30.0,
                help='DD residual threshold (m) to flag NLOS')
ap.add_argument('--min_elev',  type=float, default=10.0)
ap.add_argument('--gt_tol',    type=float, default=10.0)
ap.add_argument('--ref_tol',   type=float, default=1.0,
                help='Max time diff (s) to match reference epoch')
args = ap.parse_args()

# ── RINEX 2 obs parser ───────────────────────────────────────────────
def _rinex2_epoch_to_unix(yy, mm, dd, hh, mi, ss):
    """GPS calendar time → Unix timestamp (= GPS_time_as_unix, matching rinex_utils)."""
    year = 2000 + yy if yy < 80 else 1900 + yy
    dt   = datetime.datetime(year, mm, dd, hh, mi, int(ss),
                             int(round((ss - int(ss)) * 1e6)))
    gps_epoch = datetime.datetime(1980, 1, 6, 0, 0, 0)
    gps_sec   = (dt - gps_epoch).total_seconds()
    return gps_sec + GPS_EPOCH_UNIX

def parse_rinex2_obs(path):
    """
    Parse RINEX 2 obs file.
    Returns: (ref_ecef_xyz, epochs)
      ref_ecef_xyz : ndarray(3,) in metres, from APPROX POSITION XYZ header
      epochs       : list of dict {'rinex_t': float, 'obs': {sat_id: psr_m}}
    """
    obs_types   = []       # e.g. ['C1', 'P1', 'P2', 'L1', 'L2', ...]
    ref_ecef    = None
    epochs      = []
    in_header   = True
    i           = 0
    lines       = []

    with open(path, 'r', errors='replace') as fh:
        lines = fh.readlines()

    while i < len(lines):
        line = lines[i]
        i += 1

        if in_header:
            label = line[60:].strip()
            if 'TYPES OF OBSERV' in label:
                n_types = int(line[:6].strip() or 0)
                raw     = line[6:60]
                # up to 9 per line (6 chars each), continuation on next line
                types_so_far = [raw[j*6:(j+1)*6].strip() for j in range(min(n_types, 9))]
                obs_types.extend(t for t in types_so_far if t)
                cont_lines = math.ceil(n_types / 9) - 1
                for _ in range(cont_lines):
                    cl = lines[i]; i += 1
                    obs_types.extend(cl[6:60][j*6:(j+1)*6].strip()
                                     for j in range(9)
                                     if cl[6:60][j*6:(j+1)*6].strip())
            elif 'APPROX POSITION XYZ' in label:
                ref_ecef = np.array([float(line[0:14]),
                                     float(line[14:28]),
                                     float(line[28:42])])
            elif 'END OF HEADER' in label:
                in_header = False
            continue

        # ── epoch line ────────────────────────────────────────────────
        if len(line) < 32 or line[0] != ' ':
            continue
        try:
            yy  = int(line[1:3])
            mm  = int(line[4:6])
            dd  = int(line[7:9])
            hh  = int(line[10:12])
            mi  = int(line[13:15])
            ss  = float(line[15:26])
            flag  = int(line[26:29].strip() or 0)
            nsats = int(line[29:32].strip() or 0)
        except ValueError:
            continue

        if flag != 0 or nsats == 0:
            # skip non-OK epochs; read past their data lines
            for _ in range(nsats * math.ceil(max(len(obs_types), 1) / 5)):
                if i < len(lines):
                    i += 1
            continue

        # satellite list (3 chars each, up to 12 per line, continuation at col 32)
        sat_ids = []
        sat_part = line[32:68]
        for k in range(12):
            s = sat_part[k*3:(k+1)*3].strip()
            if s:
                sat_ids.append(s)
        while len(sat_ids) < nsats and i < len(lines):
            cont = lines[i]; i += 1
            for k in range(12):
                if len(sat_ids) >= nsats:
                    break
                s = cont[32 + k*3 : 35 + k*3].strip() if len(cont) > 32 + k*3 else ''
                if s:
                    sat_ids.append(s)

        rinex_t = _rinex2_epoch_to_unix(yy, mm, dd, hh, mi, ss)
        epoch_obs = {}
        n_lines_per_sat = math.ceil(len(obs_types) / 5) if obs_types else 1

        for raw_sid in sat_ids:
            # normalise RINEX 2 sat ID: GNSS char + 2-digit PRN
            # 'G01', 'R01', ' 1' → 'G01' or 'R01'
            raw_sid = raw_sid.strip()
            if not raw_sid:
                for _ in range(n_lines_per_sat):
                    if i < len(lines): i += 1
                continue
            if raw_sid[0].isdigit():
                # bare number → GPS
                sys_c  = 'G'
                prn    = int(raw_sid)
                sat_id = f'G{prn:02d}'
            elif len(raw_sid) >= 3 and raw_sid[0].isalpha():
                sys_c  = raw_sid[0]
                try:
                    prn = int(raw_sid[1:])
                    sat_id = f'{sys_c}{prn:02d}'
                except ValueError:
                    for _ in range(n_lines_per_sat):
                        if i < len(lines): i += 1
                    continue
            else:
                for _ in range(n_lines_per_sat):
                    if i < len(lines): i += 1
                continue

            obs_vals = []
            for _ in range(n_lines_per_sat):
                if i >= len(lines):
                    break
                ol = lines[i]; i += 1
                for k in range(5):
                    col = k * 16
                    if col >= len(ol) - 1:
                        obs_vals.append(0.0)
                        continue
                    val_s = ol[col:col+14].strip()
                    try:
                        obs_vals.append(float(val_s) if val_s else 0.0)
                    except ValueError:
                        obs_vals.append(0.0)

            # find C1 or P1 pseudorange
            psr = 0.0
            for want in ('C1', 'P1', 'C2', 'P2'):
                if want in obs_types:
                    idx = obs_types.index(want)
                    if idx < len(obs_vals) and obs_vals[idx] > 1e4:
                        psr = obs_vals[idx]; break
            if psr > 1e4:
                epoch_obs[sat_id] = psr

        if epoch_obs:
            epochs.append({'rinex_t': rinex_t, 'obs': epoch_obs})

    return ref_ecef, epochs


# ── helpers ──────────────────────────────────────────────────────────
def sagnac_correct(sat_ecef, travel_time_s):
    theta = OMEGA_E * travel_time_s
    c, s  = math.cos(theta), math.sin(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ sat_ecef

def tropo_delay(elev_deg):
    return 2.3 / math.sin(math.radians(max(elev_deg, 3.0)) + 0.017)

def elev_from_ecef(rx_ecef, sat_ecef, rx_lat, rx_lon):
    diff = sat_ecef - rx_ecef
    r    = np.linalg.norm(diff)
    lat  = math.radians(rx_lat); lon = math.radians(rx_lon)
    up   = np.array([math.cos(lat)*math.cos(lon),
                     math.cos(lat)*math.sin(lon),
                     math.sin(lat)])
    return math.degrees(math.asin(np.clip(np.dot(diff/r, up), -1, 1)))

def load_nav(path, prefix):
    try:
        raw = read_rinex_nav(path)
    except Exception as e:
        print(f'  [WARN] nav {path}: {e}'); return {}
    out = {}
    for k, v in raw.items():
        if isinstance(k, int):
            nk = f'{prefix}{k:02d}'
        elif isinstance(k, str) and k and k[0] != prefix:
            nk = prefix + k[1:]
        else:
            nk = k
        out[nk] = v
    return out

def dms_to_deg(d, m, s):
    return float(d) + float(m)/60 + float(s)/3600

PSR_KEYS_ROVER = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
    'R': ['C1C', 'C1P'],
}

# ── load ground truth ────────────────────────────────────────────────
print(f'Loading GT: {args.gt}')
gt_times, gt_lats, gt_lons, gt_alts = [], [], [], []
with open(args.gt) as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('UTC') or line.startswith('('):
            continue
        p = line.split()
        if len(p) < 10:
            continue
        try:
            t = float(p[0])
            if t < 1e9: continue
            gt_times.append(t)
            gt_lats.append(dms_to_deg(p[3], p[4], p[5]))
            gt_lons.append(dms_to_deg(p[6], p[7], p[8]))
            gt_alts.append(float(p[9]))
        except (ValueError, IndexError):
            continue

gt_times = np.array(gt_times); gt_lats = np.array(gt_lats)
gt_lons  = np.array(gt_lons);  gt_alts  = np.array(gt_alts)
order    = np.argsort(gt_times)
gt_times, gt_lats, gt_lons, gt_alts = (
    gt_times[order], gt_lats[order], gt_lons[order], gt_alts[order])
print(f'  {len(gt_times)} GT points')

def match_gt(utc_t):
    idx = int(np.searchsorted(gt_times, utc_t))
    idx = min(max(idx, 0), len(gt_times)-1)
    if idx > 0 and abs(gt_times[idx-1]-utc_t) < abs(gt_times[idx]-utc_t):
        idx -= 1
    if abs(gt_times[idx]-utc_t) > args.gt_tol:
        return None
    return gt_lats[idx], gt_lons[idx], gt_alts[idx]

# ── load HKSC reference station obs ─────────────────────────────────
print(f'Parsing reference obs: {args.ref_obs}')
ref_ecef, ref_epochs = parse_rinex2_obs(args.ref_obs)
print(f'  {len(ref_epochs)} reference epochs parsed')
if ref_ecef is not None:
    ref_lat, ref_lon, ref_alt = (
        math.degrees(math.atan2(ref_ecef[2],
            math.sqrt(ref_ecef[0]**2 + ref_ecef[1]**2))),
        math.degrees(math.atan2(ref_ecef[1], ref_ecef[0])),
        0.0)
    print(f'  HKSC position: {ref_lat:.4f}°N  {ref_lon:.4f}°E  (ECEF {ref_ecef})')
else:
    print('  [WARN] APPROX POSITION not found in reference header')
    ref_ecef = np.array([-2411594.0, 5382988.0, 2406498.0])  # HK TST fallback

# build lookup: rinex_t → obs dict (sorted for binary search)
ref_sorted_t = np.array([e['rinex_t'] for e in ref_epochs])
ref_sorted_epochs = ref_epochs  # same order

def match_ref(rinex_t):
    idx = int(np.searchsorted(ref_sorted_t, rinex_t))
    idx = min(max(idx, 0), len(ref_sorted_t)-1)
    if idx > 0 and abs(ref_sorted_t[idx-1]-rinex_t) < abs(ref_sorted_t[idx]-rinex_t):
        idx -= 1
    if abs(ref_sorted_t[idx]-rinex_t) > args.ref_tol:
        return None
    return ref_sorted_epochs[idx]['obs']

# ── load navigation + rover obs ──────────────────────────────────────
print('Loading navigation...')
ephem = {}
ephem.update(load_nav(args.nav_gps, 'G'))
print(f'  {sum(len(v) for v in ephem.values())} ephemerides for {len(ephem)} sats')

print(f'Loading rover obs: {args.obs}')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} rover epochs')

# ── load LiDAR 2-bounce NLOS labels ─────────────────────────────────
print(f'Loading LiDAR 2b labels: {args.nlos_2b}')
nlos_lidar = {}   # key: (rinex_t_str, sat_id) or (utc_t_str, sat_id) → 1
with open(args.nlos_2b) as f:
    for row in csv.DictReader(f):
        if int(row['lidar_nlos']) == 0:
            continue
        nlos_lidar[(row['unix_t'], row['sat_id'])] = 1
print(f'  {len(nlos_lidar)} LiDAR NLOS records')

# ── main DD loop ──────────────────────────────────────────────────────
print(f'\nComputing DD residuals (threshold={args.dd_thresh} m)...')
out_rows = []
n_epoch_ok = n_epoch_skip_gt = n_epoch_skip_ref = n_epoch_skip_sats = 0
n_dd_computed = n_dd_nlos = n_lidar_nlos = n_combined_nlos = 0

for ep_i, epoch in enumerate(obs_epochs):
    rinex_t = epoch.time_unix
    utc_t   = rinex_t - LEAP_SECONDS

    # match GT
    gt_match = match_gt(utc_t)
    if gt_match is None:
        n_epoch_skip_gt += 1
        continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    # match reference epoch
    ref_obs = match_ref(rinex_t)
    if ref_obs is None:
        n_epoch_skip_ref += 1
        continue

    # compute SD residuals for each GPS sat
    sd_residuals = {}  # sat_id → (sd_resid, elev, rover_psr, ref_psr)
    for obs in epoch.obs_list:
        sat_id   = obs.sat_id
        sys_char = obs.sys if obs.sys else sat_id[0]
        if sys_char != 'G':
            continue  # GPS only for now

        # rover pseudorange
        rover_psr = 0.0
        for k in PSR_KEYS_ROVER.get(sys_char, ['C1C']):
            if k in obs.pseudorange and obs.pseudorange[k] > 0:
                rover_psr = obs.pseudorange[k]; break
        if rover_psr <= 0:
            continue

        # reference pseudorange
        ref_psr = ref_obs.get(sat_id, 0.0)
        if ref_psr <= 0:
            continue

        # satellite position
        eph = find_closest_ephem(ephem.get(sat_id, []), rinex_t)
        if eph is None:
            continue
        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_t)
        if sat_ecef_raw is None:
            continue

        travel_time = rover_psr / C_LIGHT
        sat_ecef    = sagnac_correct(np.array(sat_ecef_raw), travel_time)

        # elevation from GT position
        elev = elev_from_ecef(gt_ecef, sat_ecef, gt_lat, gt_lon)
        if elev < args.min_elev:
            continue

        # geometric SD (no clock correction — cancels in DD)
        range_rover = float(np.linalg.norm(sat_ecef - gt_ecef))
        range_ref   = float(np.linalg.norm(sat_ecef - ref_ecef))
        sd_geom     = range_rover - range_ref

        # satellite clock correction (same for both rover and ref → cancels in DD)
        # tropo delay difference (small for short baseline, ~2-4 km)
        tropo_rover = tropo_delay(elev)
        ref_lat_rad = math.atan2(ref_ecef[2],
                                  math.sqrt(ref_ecef[0]**2 + ref_ecef[1]**2))
        ref_lon_rad = math.atan2(ref_ecef[1], ref_ecef[0])
        elev_ref    = elev_from_ecef(ref_ecef, sat_ecef,
                                      math.degrees(ref_lat_rad),
                                      math.degrees(ref_lon_rad))
        tropo_ref   = tropo_delay(elev_ref)

        # measured SD = rover_psr - ref_psr (satellite clock cancels)
        sd_measured = rover_psr - ref_psr

        # SD residual = measured SD − geometric SD − Δtropo
        sd_resid    = sd_measured - sd_geom - (tropo_rover - tropo_ref)

        sd_residuals[sat_id] = (sd_resid, elev, rover_psr, ref_psr)

    if len(sd_residuals) < 2:
        n_epoch_skip_sats += 1
        continue
    n_epoch_ok += 1

    # pick pivot = highest elevation GPS sat
    pivot_sat  = max(sd_residuals.keys(), key=lambda s: sd_residuals[s][1])
    pivot_sd   = sd_residuals[pivot_sat][0]

    # compute DD residuals and flag
    for sat_id, (sd_resid, elev, rover_psr, ref_psr) in sd_residuals.items():
        if sat_id == pivot_sat:
            dd_resid = 0.0
        else:
            dd_resid = sd_resid - pivot_sd

        nlos_dd = 1 if abs(dd_resid) > args.dd_thresh else 0

        # lidar label
        key_r  = (f'{rinex_t:.3f}', sat_id)
        key_u  = (f'{utc_t:.3f}',   sat_id)
        nlos_l = 1 if (nlos_lidar.get(key_r) or nlos_lidar.get(key_u)) else 0

        nlos_comb = 1 if (nlos_dd and nlos_l) else 0

        n_dd_computed += 1
        if nlos_dd:    n_dd_nlos      += 1
        if nlos_l:     n_lidar_nlos   += 1
        if nlos_comb:  n_combined_nlos+= 1

        out_rows.append({
            'rinex_t':  f'{rinex_t:.3f}',
            'utc_t':    f'{utc_t:.3f}',
            'sat_id':   sat_id,
            'elev_deg': f'{elev:.2f}',
            'sd_resid': f'{sd_resid:.3f}',
            'dd_resid': f'{dd_resid:.3f}',
            'nlos_dd':  nlos_dd,
            'nlos_lidar': nlos_l,
            'nlos_combined': nlos_comb,
        })

print(f'\n=== Epoch summary ===')
print(f'  GT-matched epochs:           {n_epoch_ok + n_epoch_skip_gt + n_epoch_skip_ref + n_epoch_skip_sats}')
print(f'  Skipped (no GT):             {n_epoch_skip_gt}')
print(f'  Skipped (no ref match):      {n_epoch_skip_ref}')
print(f'  Skipped (<2 common GPS sat): {n_epoch_skip_sats}')
print(f'  Epochs processed:            {n_epoch_ok}')
print(f'\n=== NLOS label summary (threshold={args.dd_thresh} m) ===')
print(f'  Total sat-obs computed:      {n_dd_computed}')
print(f'  DD-flagged NLOS:             {n_dd_nlos}  ({100*n_dd_nlos/max(n_dd_computed,1):.1f}%)')
print(f'  LiDAR-flagged NLOS:          {n_lidar_nlos}  ({100*n_lidar_nlos/max(n_dd_computed,1):.1f}%)')
print(f'  Combined (both) NLOS:        {n_combined_nlos}  ({100*n_combined_nlos/max(n_dd_computed,1):.1f}%)')

# confusion matrix between DD and LiDAR
tp = sum(1 for r in out_rows if r['nlos_dd']==1 and r['nlos_lidar']==1)
fp = sum(1 for r in out_rows if r['nlos_dd']==1 and r['nlos_lidar']==0)
fn = sum(1 for r in out_rows if r['nlos_dd']==0 and r['nlos_lidar']==1)
tn = sum(1 for r in out_rows if r['nlos_dd']==0 and r['nlos_lidar']==0)
print(f'\n=== Confusion matrix: DD vs LiDAR (treating LiDAR as reference) ===')
print(f'  TP (both say NLOS):           {tp}')
print(f'  FP (DD says NLOS, LiDAR LOS): {fp}')
print(f'  FN (LiDAR NLOS, DD says LOS): {fn}')
print(f'  TN (both say LOS):            {tn}')
prec = tp/(tp+fp) if (tp+fp)>0 else 0
rec  = tp/(tp+fn) if (tp+fn)>0 else 0
f1   = 2*prec*rec/(prec+rec) if (prec+rec)>0 else 0
print(f'  Precision (of DD NLOS):       {prec:.3f}')
print(f'  Recall    (of DD NLOS):       {rec:.3f}')
print(f'  F1 score:                     {f1:.3f}')

# ── save CSV ──────────────────────────────────────────────────────────
with open(args.out_csv, 'w', newline='') as f:
    if out_rows:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        w.writeheader(); w.writerows(out_rows)
print(f'\nSaved CSV: {args.out_csv}  ({len(out_rows)} rows)')

# ── plots ─────────────────────────────────────────────────────────────
def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=120, bbox_inches='tight')
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode()

figs_b64 = []

# 1. DD residual histogram by elevation bin
if out_rows:
    dd_vals    = np.array([float(r['dd_resid']) for r in out_rows])
    elev_vals  = np.array([float(r['elev_deg']) for r in out_rows])
    nlos_dd_m  = np.array([r['nlos_dd']     for r in out_rows])
    nlos_l_m   = np.array([r['nlos_lidar']  for r in out_rows])

    bins_el = [(10,20),(20,30),(30,45),(45,60),(60,90)]
    fig, axes = plt.subplots(1, len(bins_el), figsize=(15, 4), sharey=False)
    fig.suptitle('DD Residual Distribution by Elevation Bin', fontsize=12)
    for ax, (lo, hi) in zip(axes, bins_el):
        mask = (elev_vals >= lo) & (elev_vals < hi)
        vals = dd_vals[mask]
        if len(vals) == 0:
            ax.set_title(f'{lo}-{hi}°\n(no data)')
            continue
        ax.hist(vals, bins=30, color='steelblue', edgecolor='white', alpha=0.8)
        ax.axvline(args.dd_thresh,  color='red',    lw=1.5, ls='--', label=f'+{args.dd_thresh}m')
        ax.axvline(-args.dd_thresh, color='red',    lw=1.5, ls='--')
        ax.axvline(0, color='gray', lw=1, ls=':')
        nlos_frac = nlos_dd_m[mask].mean()
        ax.set_title(f'{lo}-{hi}°  n={len(vals)}\nDD-NLOS: {nlos_frac:.1%}')
        ax.set_xlabel('DD residual (m)')
    axes[0].set_ylabel('Count')
    plt.tight_layout()
    figs_b64.append(fig_to_b64(fig))

    # 2. Confusion scatter: DD residual vs elevation, coloured by LiDAR label
    fig, ax = plt.subplots(figsize=(9, 5))
    for label, color, marker in [(0, '#2196F3', 'o'), (1, '#F44336', 'x')]:
        mask = nlos_l_m == label
        ax.scatter(elev_vals[mask], dd_vals[mask],
                   c=color, marker=marker, s=10, alpha=0.4,
                   label='LiDAR LOS' if label==0 else 'LiDAR NLOS')
    ax.axhline(args.dd_thresh,  color='black', lw=1.5, ls='--', label=f'±{args.dd_thresh}m threshold')
    ax.axhline(-args.dd_thresh, color='black', lw=1.5, ls='--')
    ax.set_xlabel('Elevation (deg)')
    ax.set_ylabel('DD residual (m)')
    ax.set_title('DD Residual vs Elevation  (coloured by LiDAR label)')
    ax.legend(markerscale=2, fontsize=9)
    ax.set_ylim(-300, 300)
    plt.tight_layout()
    figs_b64.append(fig_to_b64(fig))

    # 3. Venn-style NLOS agreement bar chart
    cats  = ['LiDAR only\n(FN_dd)', 'DD only\n(FP_dd)', 'Both\n(TP)', 'Neither\n(TN)']
    cnts  = [fn, fp, tp, tn]
    colors = ['#FF9800','#F44336','#4CAF50','#90CAF9']
    fig, ax = plt.subplots(figsize=(7, 4))
    bars = ax.bar(cats, cnts, color=colors, edgecolor='white')
    for b, c in zip(bars, cnts):
        ax.text(b.get_x()+b.get_width()/2, b.get_height()+5, str(c),
                ha='center', va='bottom', fontsize=10)
    ax.set_ylabel('Satellite observations')
    ax.set_title(f'DD vs LiDAR NLOS Agreement  (threshold={args.dd_thresh} m)')
    plt.tight_layout()
    figs_b64.append(fig_to_b64(fig))

    # 4. DD residual CDF for LiDAR-NLOS vs LiDAR-LOS
    fig, ax = plt.subplots(figsize=(7, 4))
    for label, color, lbl in [(0,'#2196F3','LiDAR LOS'), (1,'#F44336','LiDAR NLOS')]:
        mask = nlos_l_m == label
        vals = np.sort(np.abs(dd_vals[mask]))
        if len(vals):
            ax.plot(vals, np.linspace(0,1,len(vals)), color=color, lw=2, label=lbl)
    ax.axvline(args.dd_thresh, color='black', ls='--', label=f'{args.dd_thresh}m threshold')
    ax.set_xlabel('|DD residual| (m)')
    ax.set_ylabel('CDF')
    ax.set_title('|DD Residual| CDF: LiDAR-LOS vs LiDAR-NLOS')
    ax.legend(); ax.set_xlim(0, 300)
    plt.tight_layout()
    figs_b64.append(fig_to_b64(fig))

# ── HTML report ──────────────────────────────────────────────────────
img_html = '\n'.join(
    f'<img src="data:image/png;base64,{b}" style="max-width:100%;margin:12px 0">'
    for b in figs_b64)

html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>DD NLOS Pre-screening Report</title>
<style>
  body{{font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;margin:20px}}
  h1{{color:#64b5f6}} h2{{color:#90caf9;border-bottom:1px solid #444;padding-bottom:6px}}
  table{{border-collapse:collapse;margin:10px 0}}
  td,th{{border:1px solid #555;padding:6px 12px}}
  th{{background:#263238}} .good{{color:#4CAF50}} .warn{{color:#FF9800}} .bad{{color:#F44336}}
</style>
</head>
<body>
<h1>DD NLOS Pre-screening — HKSC Reference Station</h1>

<h2>Configuration</h2>
<table>
  <tr><th>Parameter</th><th>Value</th></tr>
  <tr><td>DD threshold</td><td>{args.dd_thresh} m</td></tr>
  <tr><td>Min elevation</td><td>{args.min_elev}°</td></tr>
  <tr><td>Reference match tolerance</td><td>±{args.ref_tol} s</td></tr>
  <tr><td>Constellations used</td><td>GPS (G)</td></tr>
</table>

<h2>Epoch Statistics</h2>
<table>
  <tr><th>Metric</th><th>Count</th></tr>
  <tr><td>Epochs processed (DD computed)</td><td>{n_epoch_ok}</td></tr>
  <tr><td>Skipped — no GT match</td><td>{n_epoch_skip_gt}</td></tr>
  <tr><td>Skipped — no reference epoch</td><td>{n_epoch_skip_ref}</td></tr>
  <tr><td>Skipped — &lt;2 common GPS sats</td><td>{n_epoch_skip_sats}</td></tr>
</table>

<h2>NLOS Label Comparison</h2>
<table>
  <tr><th>Label Source</th><th>NLOS count</th><th>NLOS rate</th></tr>
  <tr><td>DD flagged (|DD resid| &gt; {args.dd_thresh}m)</td>
      <td>{n_dd_nlos}</td><td>{100*n_dd_nlos/max(n_dd_computed,1):.1f}%</td></tr>
  <tr><td>LiDAR ray casting</td>
      <td>{n_lidar_nlos}</td><td>{100*n_lidar_nlos/max(n_dd_computed,1):.1f}%</td></tr>
  <tr><td>Combined (both agree NLOS)</td>
      <td>{n_combined_nlos}</td><td>{100*n_combined_nlos/max(n_dd_computed,1):.1f}%</td></tr>
</table>

<h2>DD vs LiDAR Confusion Matrix</h2>
<table>
  <tr><th></th><th>LiDAR: LOS</th><th>LiDAR: NLOS</th></tr>
  <tr><th>DD: LOS</th><td class="good">{tn} (TN)</td><td class="warn">{fn} (FN)</td></tr>
  <tr><th>DD: NLOS</th><td class="warn">{fp} (FP)</td><td class="good">{tp} (TP)</td></tr>
</table>
<p>
  Precision (of DD NLOS label): <b>{prec:.3f}</b><br>
  Recall of LiDAR NLOS by DD:   <b>{rec:.3f}</b><br>
  F1 score:                      <b>{f1:.3f}</b>
</p>
<p><i>Interpretation: High precision = DD NLOS flags are reliable.
High recall = DD captures what LiDAR labels as NLOS.
Low precision + low recall = the two methods see different things.</i></p>

<h2>Visualisations</h2>
{img_html}

<h2>Key Takeaways</h2>
<ul>
  <li>DD residual is a signal-domain consistency check; LiDAR ray casting is a geometry check.</li>
  <li>Combined NLOS (both agree): lowest false-positive rate — safest for SPP correction.</li>
  <li>DD-only NLOS: either a multipath case LiDAR misses (e.g., ground reflection), or false alarm.</li>
  <li>LiDAR-only NLOS: ray-casting hit a building, but the signal may have diffracted (false positive).</li>
</ul>
</body>
</html>"""

with open(args.out_html, 'w') as f:
    f.write(html)
print(f'Saved HTML: {args.out_html}')
