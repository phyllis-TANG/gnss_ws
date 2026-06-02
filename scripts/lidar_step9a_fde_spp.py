#!/usr/bin/env python3
"""
lidar_step9a_fde_spp.py
SPP with FDE (Fault Detection & Exclusion) + optional planarity-weighted LiDAR ΔL.

Improvements over Step 8b:
  1. FDE/RAIM: after WLS convergence, compute post-fit pseudorange residuals.
     Exclude the worst outlier if |res| > fde_sigma × σ_MAD, re-solve.
     Repeat up to fde_max_excl times per epoch.
     Catches gross outliers that DD correction did not address (non-GPS sats,
     or GPS sats with extreme multipath but small dd_resid).

  2. Planarity-weighted LiDAR ΔL (if planarity column present in --lidar_csv):
     Instead of hard-subtracting the geometric path excess ΔL from all NLOS sats,
     scale it by planarity: ΔL_eff = ΔL_geom × planarity.
     Surfaces with planarity < ~0.5 (trees, cars) get reduced correction,
     limiting damage from unreliable normal estimates.
     Requires re-running lidar_step4_ray_casting.py (now outputs planarity column).

Modes
-----
  baseline       — elevation-weighted WLS, no NLOS treatment (reference)
  dd_corr        — GPS DD-NLOS: psr -= dd_resid, t=5m   (Step 7e best, reference)
  fde            — FDE/RAIM only, no DD correction       (isolated FDE contribution)
  dd_fde         — DD correction (t=5m) + FDE            (primary new result)
  plan_fde       — planarity-weighted LiDAR ΔL + FDE     (requires planarity in lidar_csv)

Usage (inside container)
------------------------
# FDE only (no new data needed):
python3 lidar_step9a_fde_spp.py \\
  --dd_labels /root/dd_nlos_labels.csv \\
  --gt        /root/urbannav_gt.txt \\
  --out_csv   /root/spp_fde_results.csv \\
  --out_html  /root/spp_fde_report.html

# With planarity (after re-running Step 4 to get planarity column):
python3 lidar_step9a_fde_spp.py \\
  --dd_labels  /root/dd_nlos_labels.csv \\
  --lidar_csv  /root/lidar_nlos_prediction_2b.csv \\
  --gt         /root/urbannav_gt.txt \\
  --out_csv    /root/spp_fde_results.csv \\
  --out_html   /root/spp_fde_report.html
"""

import argparse, base64, csv, io, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in [
    '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts',
    '/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts',
    os.path.join(SCRIPT_DIR, '../src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts'),
]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rinex_utils import (read_rinex_obs, read_rinex_nav,
                          compute_sat_position, find_closest_ephem,
                          llh_to_ecef, ecef_to_llh)

LEAP_SECONDS = 18
C_LIGHT      = 299792458.0
OMEGA_E      = 7.2921151467e-5

PSR_KEYS = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
}

ap = argparse.ArgumentParser()
ap.add_argument('--obs',         default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps',     default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds',     default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal',     default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--dd_labels',   default='/root/dd_nlos_labels.csv')
ap.add_argument('--lidar_csv',   default='/root/lidar_nlos_prediction_2b.csv',
                help='LiDAR ray-tracing CSV; needs planarity column for plan_fde mode')
ap.add_argument('--gt',          default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',     default='/root/spp_fde_results.csv')
ap.add_argument('--out_html',    default='/root/spp_fde_report.html')
ap.add_argument('--min_elev',    type=float, default=10.0)
ap.add_argument('--gt_tol',      type=float, default=10.0)
ap.add_argument('--dd_thresh',   type=float, default=5.0,
                help='|dd_resid| threshold (m) to flag DD-NLOS (default 5m, Step 7e optimum)')
ap.add_argument('--fde_sigma',   type=float, default=3.0,
                help='FDE exclusion threshold in units of MAD-sigma (default 3.0)')
ap.add_argument('--fde_max_excl',type=int,   default=3,
                help='Max satellites to exclude per epoch via FDE (default 3)')
ap.add_argument('--plan_thresh', type=float, default=0.0,
                help='Planarity below this value → ΔL set to 0 (default 0, use all)')
args = ap.parse_args()

# ── helpers ──────────────────────────────────────────────────────────────
def load_nav(path, prefix):
    try:
        raw = read_rinex_nav(path)
    except Exception as e:
        print(f'  [WARN] {path}: {e}'); return {}
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

def sagnac_correct(sat_ecef, travel_time_s):
    theta = OMEGA_E * travel_time_s
    c, s  = math.cos(theta), math.sin(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ sat_ecef

def tropo_delay(elev_deg):
    return 2.3 / math.sin(math.radians(max(elev_deg, 3.0)) + 0.017)

def ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon):
    lat = math.radians(ref_lat); lon = math.radians(ref_lon)
    R = np.array([[-math.sin(lon), math.cos(lon), 0],
                  [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
                  [ math.cos(lat)*math.cos(lon),  math.cos(lat)*math.sin(lon), math.sin(lat)]])
    return R @ (ecef - ref_ecef)

def dms_to_deg(d, m, s):
    return float(d) + float(m)/60 + float(s)/3600

# ── WLS SPP solver ────────────────────────────────────────────────────────
def wls_spp_once(active, x_init, min_sats=4, max_iter=10):
    """Single WLS solve (no FDE). Returns (pos, clk_dict, pdop, H_final)."""
    sys_present = sorted(set(s['sys'] for s in active))
    n_clk = len(sys_present)
    if len(active) < max(min_sats, 3 + n_clk):
        return None, None, None, None
    clk_col = {sys: 3+i for i, sys in enumerate(sys_present)}
    n_unk = 3 + n_clk
    x = np.array(x_init, dtype=float)
    if len(x) < n_unk:
        x = np.zeros(n_unk); x[:3] = x_init[:3]
    H = None
    for _ in range(max_iter):
        H_list, dp_list, w_list = [], [], []
        for s in active:
            diff = x[:3] - np.array(s['sat_ecef'])
            r = np.linalg.norm(diff)
            if r < 1e4: continue
            e = diff / r
            row = np.zeros(n_unk); row[:3] = e
            row[clk_col[s['sys']]] = 1.0
            H_list.append(row)
            dp_list.append(s['psr_corr'] - r - x[clk_col[s['sys']]])
            w_list.append(s['weight'])
        if len(H_list) < max(min_sats, 3 + n_clk):
            return None, None, None, None
        H = np.array(H_list); dp = np.array(dp_list); W = np.diag(w_list)
        HtW = H.T @ W
        try:
            delta = np.linalg.solve(HtW @ H, HtW @ dp)
        except np.linalg.LinAlgError:
            return None, None, None, None
        x += delta
        if np.linalg.norm(delta[:3]) < 0.01: break
    try:
        Q = np.linalg.inv(H.T @ H)
        pdop = math.sqrt(max(Q[0,0]+Q[1,1]+Q[2,2], 0))
    except Exception:
        pdop = float('nan')
    clk_dict = {sys: float(x[col]) for sys, col in clk_col.items()}
    return x[:3], clk_dict, pdop, (H, clk_col)


def wls_spp_fde(sats_info, x0_ecef, min_sats=4, max_iter=10,
                fde_sigma=3.0, fde_max_excl=3):
    """WLS SPP with iterative FDE (MAD-based outlier exclusion).

    Returns (pos, clk_dict, pdop, n_excluded).
    After convergence, computes post-fit residuals; if the worst outlier
    exceeds fde_sigma × σ_MAD, it is excluded and the solver re-runs.
    Stops when no outlier found or fde_max_excl exclusions reached.
    """
    active = [s for s in sats_info if s['weight'] > 0]
    if not active:
        return None, None, None, 0

    x = np.zeros(6); x[:3] = x0_ecef  # enough room for up to 3 clocks
    n_excluded = 0

    for _ in range(fde_max_excl + 1):
        pos, clk, pdop, aux = wls_spp_once(active, x, min_sats=min_sats, max_iter=max_iter)
        if pos is None:
            return None, None, None, n_excluded
        x[:3] = pos

        # Post-fit residuals (absolute ranging error per satellite)
        resids = []
        for s in active:
            r_geom = np.linalg.norm(pos - np.array(s['sat_ecef']))
            res    = abs(s['psr_corr'] - r_geom - clk.get(s['sys'], 0.0))
            resids.append(res)
        resids = np.array(resids)

        # Robust sigma from MAD; floor at 5m to avoid over-aggressive exclusion
        mad   = float(np.median(np.abs(resids - np.median(resids))))
        sigma = max(mad * 1.4826, 5.0)

        worst_idx = int(np.argmax(resids))
        if resids[worst_idx] > fde_sigma * sigma and n_excluded < fde_max_excl:
            active.pop(worst_idx)
            n_excluded += 1
        else:
            break  # no outlier found → solution accepted

    return pos, clk, pdop, n_excluded


# ── load DD labels ────────────────────────────────────────────────────────
print(f'Loading DD labels: {args.dd_labels}')
dd_map = {}
with open(args.dd_labels) as f:
    for row in csv.DictReader(f):
        if row['sys'] != 'G': continue
        t_key = round(float(row['rinex_t']), 3)
        dd_map[(t_key, row['sat_id'])] = {
            'nlos_dd':  int(row['nlos_dd']),
            'dd_resid': float(row['dd_resid']),
        }
n_dd_nlos = sum(1 for v in dd_map.values() if v['nlos_dd'])
print(f'  {len(dd_map)} GPS DD records  ({n_dd_nlos} flagged at 30m; '
      f'{args.dd_thresh}m threshold used at runtime)')

# ── load LiDAR CSV (optional; for planarity-weighted ΔL) ─────────────────
# Columns: unix_t, sat_id, lidar_nlos, hit_dist_m, n_bounces,
#          normal_e, normal_n, normal_u, planarity (if re-ran Step 4)
print(f'Loading LiDAR CSV: {args.lidar_csv}')
lidar_map    = {}   # (t_key, sat_id) -> dict
has_planarity = False
try:
    with open(args.lidar_csv) as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        has_planarity = 'planarity' in fieldnames
        for row in reader:
            t_key = round(float(row.get('unix_t', row.get('rinex_t', 0))), 3)
            sid   = row.get('sat_id', '')
            lidar_map[(t_key, sid)] = {
                'lidar_nlos':  int(float(row.get('lidar_nlos', 0))),
                'hit_dist_m':  float(row.get('hit_dist_m', 0)),
                'n_bounces':   int(float(row.get('n_bounces', 0))),
                'azimuth_deg': float(row.get('azimuth_deg', 0)),
                'normal_e':    float(row.get('normal_e', 0)),
                'normal_n':   float(row.get('normal_n', 0)),
                'normal_u':   float(row.get('normal_u', 0)),
                'planarity':  float(row.get('planarity', 0)) if has_planarity else 0.0,
            }
    print(f'  {len(lidar_map)} LiDAR records  '
          f'(planarity column: {"YES" if has_planarity else "NO — re-run Step 4 to enable plan_fde mode"})')
except FileNotFoundError:
    print(f'  [WARN] {args.lidar_csv} not found — plan_fde mode disabled')

plan_mode_available = has_planarity and len(lidar_map) > 0

# ── load GT ───────────────────────────────────────────────────────────────
print(f'Loading GT: {args.gt}')
gt_times, gt_lats, gt_lons, gt_alts = [], [], [], []
with open(args.gt) as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('UTC') or line.startswith('('): continue
        p = line.split()
        if len(p) < 10: continue
        try:
            t = float(p[0])
            if t < 1e9: continue
            gt_times.append(t)
            gt_lats.append(dms_to_deg(p[3],p[4],p[5]))
            gt_lons.append(dms_to_deg(p[6],p[7],p[8]))
            gt_alts.append(float(p[9]))
        except (ValueError, IndexError): continue
gt_times = np.array(gt_times); gt_lats = np.array(gt_lats)
gt_lons  = np.array(gt_lons);  gt_alts = np.array(gt_alts)
order = np.argsort(gt_times)
gt_times,gt_lats,gt_lons,gt_alts = (
    gt_times[order],gt_lats[order],gt_lons[order],gt_alts[order])
print(f'  {len(gt_times)} GT points')

def match_gt(utc_t):
    idx = int(np.searchsorted(gt_times, utc_t))
    idx = min(max(idx,0), len(gt_times)-1)
    if idx>0 and abs(gt_times[idx-1]-utc_t)<abs(gt_times[idx]-utc_t): idx-=1
    return (gt_lats[idx],gt_lons[idx],gt_alts[idx]) \
           if abs(gt_times[idx]-utc_t)<=args.gt_tol else None

# ── load nav + obs ────────────────────────────────────────────────────────
print('Loading navigation messages...')
ephem = {}
ephem.update(load_nav(args.nav_gps, 'G'))
ephem.update(load_nav(args.nav_bds, 'C'))
ephem.update(load_nav(args.nav_gal, 'E'))
print(f'  {sum(len(v) for v in ephem.values())} ephemerides')

print(f'Loading obs: {args.obs}')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} epochs')

x0_ecef = np.array(llh_to_ecef(22.3198, 114.2095, 20.0))

# Decide active modes based on data availability
MODES = ['baseline', 'dd_corr', 'fde', 'dd_fde']
if plan_mode_available:
    MODES.append('plan_fde')
    print(f'  plan_fde mode ENABLED (planarity column found)')
else:
    print(f'  plan_fde mode DISABLED (re-run lidar_step4_ray_casting.py to enable)')

# ── main loop ──────────────────────────────────────────────────────────────
print(f'\nRunning SPP ({len(MODES)} modes, dd_thresh={args.dd_thresh}m, '
      f'fde_sigma={args.fde_sigma}σ, fde_max_excl={args.fde_max_excl})...')
results   = []
no_gt     = 0
fde_excl_total = {m: 0 for m in MODES}

for ep_i, epoch in enumerate(obs_epochs):
    rinex_t     = epoch.time_unix
    utc_t       = rinex_t - LEAP_SECONDS
    rinex_t_key = round(rinex_t, 3)

    gt_match = match_gt(utc_t)
    if gt_match is None:
        no_gt += 1; continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    sat_data = []
    for obs in epoch.obs_list:
        sat_id   = obs.sat_id
        sys_char = obs.sys if obs.sys else sat_id[0]
        if sys_char not in ('G', 'C', 'E'): continue
        eph = find_closest_ephem(ephem.get(sat_id, []), rinex_t)
        if eph is None: continue
        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_t)
        if sat_ecef_raw is None: continue

        psr_raw = 0.0
        for k in PSR_KEYS.get(sys_char, ['C1C']):
            if k in obs.pseudorange and obs.pseudorange[k] > 0:
                psr_raw = obs.pseudorange[k]; break
        if psr_raw <= 0: continue

        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), psr_raw/C_LIGHT)
        diff_rx    = sat_ecef_s - x0_ecef
        r_approx   = np.linalg.norm(diff_rx)
        lat_r, lon_r = math.radians(gt_lat), math.radians(gt_lon)
        up = np.array([math.cos(lat_r)*math.cos(lon_r),
                       math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)])
        elev = math.degrees(math.asin(np.clip(np.dot(diff_rx/r_approx, up), -1, 1)))
        if elev < args.min_elev: continue

        psr_corr = psr_raw + C_LIGHT*dt_sv - tropo_delay(elev)
        w_base   = math.sin(math.radians(elev))**2

        # DD info (GPS only)
        dd_info  = dd_map.get((rinex_t_key, sat_id))
        dd_resid = float(dd_info['dd_resid']) if (dd_info and sys_char=='G') else 0.0
        nlos_dd  = 1 if (dd_info and sys_char=='G'
                         and abs(dd_resid) > args.dd_thresh) else 0

        # LiDAR geometric ΔL (for plan_fde mode)
        lidar_info = lidar_map.get((rinex_t_key, sat_id))
        delta_L    = 0.0
        planarity  = 0.0
        if lidar_info and lidar_info['lidar_nlos'] and lidar_info['n_bounces'] >= 1:
            d_hit = lidar_info['hit_dist_m']
            ne, nn, nu = lidar_info['normal_e'], lidar_info['normal_n'], lidar_info['normal_u']
            az_r  = math.radians(lidar_info['azimuth_deg'])
            el_r  = math.radians(elev)
            cos_el = math.cos(el_r)
            # incoming signal unit vector (receiver→satellite direction)
            d_vec = np.array([math.sin(az_r)*cos_el,
                              math.cos(az_r)*cos_el,
                              math.sin(el_r)])
            norm_vec = np.array([ne, nn, nu])
            norm_len = np.linalg.norm(norm_vec)
            if norm_len > 0.1:
                norm_vec /= norm_len
                cos_inc = abs(float(np.dot(d_vec, norm_vec)))
                delta_L   = 2.0 * d_hit * cos_inc
                planarity = lidar_info['planarity']
                # planarity weighting: scale ΔL by surface quality
                if has_planarity:
                    delta_L *= max(planarity, args.plan_thresh)

        sat_data.append({
            'sat_id':    sat_id,
            'sys':       sys_char,
            'sat_ecef':  sat_ecef_s,
            'psr_corr':  psr_corr,
            'elev':      elev,
            'w_base':    w_base,
            'nlos_dd':   nlos_dd,
            'dd_resid':  dd_resid,
            'delta_L':   delta_L,
            'planarity': planarity,
        })

    if len(sat_data) < 4: continue

    mode_results = {}
    for mode in MODES:
        sats_info = []
        for s in sat_data:
            psr = s['psr_corr']
            w   = s['w_base']

            if mode == 'dd_corr' or mode == 'dd_fde':
                if s['nlos_dd']:
                    psr = s['psr_corr'] - s['dd_resid']

            elif mode == 'plan_fde':
                if s['delta_L'] > 0:
                    psr = s['psr_corr'] - s['delta_L']

            sats_info.append({
                'sat_ecef': s['sat_ecef'],
                'psr_corr': psr,
                'weight':   w,
                'sys':      s['sys'],
            })

        use_fde = mode in ('fde', 'dd_fde', 'plan_fde')
        if use_fde:
            pos, clk, pdop, n_excl = wls_spp_fde(
                sats_info, x0_ecef,
                fde_sigma=args.fde_sigma, fde_max_excl=args.fde_max_excl)
            fde_excl_total[mode] += n_excl
        else:
            pos, clk, pdop, _ = wls_spp_once(
                [s for s in sats_info if s['weight']>0], x0_ecef)
            if pos is None:
                mode_results[mode] = None; continue

        if pos is None:
            mode_results[mode] = None; continue

        enu   = ecef_to_enu(pos, gt_ecef, gt_lat, gt_lon)
        err_h = math.sqrt(enu[0]**2 + enu[1]**2)
        err_v = abs(enu[2])
        mode_results[mode] = {
            'err_h':  round(err_h, 2),
            'err_v':  round(err_v, 2),
            'n_used': sum(1 for si in sats_info if si['weight']>0),
            'pdop':   round(pdop, 2) if not math.isnan(pdop) else None,
        }

    if all(v is None for v in mode_results.values()): continue

    results.append({
        'utc_t':  round(utc_t, 3),
        'n_total': len(sat_data),
        'n_dd':    sum(1 for s in sat_data if s['nlos_dd']),
        **{f'{m}_{k}': (mode_results[m][k] if mode_results[m] else None)
           for m in MODES for k in ('err_h','err_v','n_used','pdop')},
    })

    if (ep_i+1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} epochs, {len(results)} valid...',
              end='', flush=True)

print(f'\nDone: {len(results)} valid epochs  (no_gt={no_gt})')
for m in MODES:
    if m in ('fde','dd_fde','plan_fde'):
        avg = fde_excl_total[m] / max(len(results),1)
        print(f'  FDE exclusions [{m}]: total={fde_excl_total[m]}, avg={avg:.2f}/epoch')

# ── statistics ─────────────────────────────────────────────────────────────
def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return dict(n=0, mean=float('nan'), rms=float('nan'),
                    p50=float('nan'), p95=float('nan'))
    a = np.array(v)
    return dict(n=len(v), mean=float(np.mean(a)),
                rms=float(np.sqrt(np.mean(a**2))),
                p50=float(np.percentile(a,50)),
                p95=float(np.percentile(a,95)))

st = {m: stats([r[f'{m}_err_h'] for r in results]) for m in MODES}

print('\n=== Horizontal Error Summary ===')
print(f'  {"Mode":<12}  {"n":>4}  {"mean":>7}  {"RMS":>7}  {"50th":>7}  '
      f'{"95th":>7}  vs-baseline  vs-dd_corr')
for m in MODES:
    s  = st[m]
    d0 = f'{s["mean"]-st["baseline"]["mean"]:+.2f}m' if m!='baseline' else '—'
    d1 = f'{s["mean"]-st["dd_corr"]["mean"]:+.2f}m'  if m!='dd_corr'  else '—'
    print(f'  {m:<12}  {s["n"]:4d}  {s["mean"]:7.2f}m  {s["rms"]:7.2f}m  '
          f'{s["p50"]:7.2f}m  {s["p95"]:7.2f}m  {d0:<12}  {d1}')

# ── save CSV ───────────────────────────────────────────────────────────────
COLS = ['utc_t','n_total','n_dd'] + \
       [f'{m}_{k}' for m in MODES for k in ('err_h','err_v','n_used','pdop')]
with open(args.out_csv,'w',newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS, extrasaction='ignore')
    w.writeheader(); w.writerows(results)
print(f'Saved CSV: {args.out_csv}')

# ── plots ──────────────────────────────────────────────────────────────────
def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

colors = {
    'baseline': '#F44336',
    'dd_corr':  '#2196F3',
    'fde':      '#FF9800',
    'dd_fde':   '#4CAF50',
    'plan_fde': '#9C27B0',
}
labels_m = {
    'baseline': 'Baseline',
    'dd_corr':  f'DD-corr t={args.dd_thresh}m (Step 7e ref)',
    'fde':      f'FDE σ={args.fde_sigma} (isolated)',
    'dd_fde':   f'DD-corr + FDE  ← primary',
    'plan_fde': f'Planarity-ΔL + FDE',
}
imgs = {}

# Fig 1: CDF
fig, ax = plt.subplots(figsize=(9,5))
for m in MODES:
    vals = sorted(r[f'{m}_err_h'] for r in results if r[f'{m}_err_h'] is not None)
    if vals:
        lw = 2.8 if m=='dd_fde' else 1.6
        ax.plot(vals, np.linspace(0,1,len(vals)),
                color=colors[m], lw=lw,
                label=f'{labels_m[m]}  mean={st[m]["mean"]:.2f}m')
ax.set_xlabel('Horizontal error (m)'); ax.set_ylabel('CDF')
ax.set_title('SPP Horizontal Error CDF — FDE + DD Correction')
ax.set_xlim(0,200); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
imgs['cdf'] = fig_to_b64(fig)

# Fig 2: bar
fig, ax = plt.subplots(figsize=(10,5))
metrics = ['mean','rms','p50','p95']
mlabels = ['Mean','RMS','50th pct','95th pct']
x = np.arange(len(metrics)); bw = 0.8/len(MODES)
for i,m in enumerate(MODES):
    vals = [st[m][k] for k in metrics]
    ax.bar(x + i*bw, vals, bw, label=labels_m[m], color=colors[m], alpha=0.85,
           edgecolor='white')
ax.set_xticks(x + bw*(len(MODES)-1)/2); ax.set_xticklabels(mlabels)
ax.set_ylabel('Error (m)'); ax.set_title('SPP Horizontal Error by Mode')
ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
imgs['bar'] = fig_to_b64(fig)

# Fig 3: time series
fig, ax = plt.subplots(figsize=(13,4.5))
for m in MODES:
    ts = [(r['utc_t'],r[f'{m}_err_h']) for r in results if r[f'{m}_err_h'] is not None]
    if ts:
        lw = 2.2 if m=='dd_fde' else 1.0
        ax.plot([t for t,_ in ts],[e for _,e in ts],
                color=colors[m], lw=lw, alpha=0.8, label=labels_m[m])
ax.set_xlabel('UTC time (s)'); ax.set_ylabel('Horizontal error (m)')
ax.set_title('SPP Horizontal Error — Time Series'); ax.set_ylim(0,300)
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
imgs['ts'] = fig_to_b64(fig)

# Fig 4: per-epoch dd_fde vs baseline improvement
fig, ax = plt.subplots(figsize=(13,4))
paired = [(r['utc_t'], r['baseline_err_h']-r['dd_fde_err_h'])
          for r in results
          if r['baseline_err_h'] is not None and r['dd_fde_err_h'] is not None]
if paired:
    ts_p = [t for t,_ in paired]; impr = [d for _,d in paired]
    pos_frac = 100*sum(1 for d in impr if d>0)/len(impr)
    ax.bar(ts_p, impr, width=2,
           color=['#4CAF50' if d>0 else '#F44336' for d in impr], alpha=0.75)
    ax.axhline(0, color='k', lw=0.8)
    ax.set_xlabel('UTC time (s)')
    ax.set_ylabel('Improvement (m)\n(positive = dd_fde better)')
    ax.set_title(f'Per-epoch improvement: DD+FDE vs Baseline  '
                 f'(better in {pos_frac:.0f}% of epochs)')
    ax.grid(axis='y', alpha=0.3)
imgs['improvement'] = fig_to_b64(fig)

# ── HTML ───────────────────────────────────────────────────────────────────
def pct_change(m):
    d  = st[m]['mean'] - st['baseline']['mean']
    ar = '&darr;' if d<0 else '&uarr;'
    cl = 'better' if d<0 else 'worse'
    return f'<span class="{cl}">{ar}{abs(d):.2f}m</span>'

best = min(MODES, key=lambda m: st[m]['mean'])

plan_note = (
    '<p style="color:#6a1b9a">✔ plan_fde mode active '
    f'(planarity column found, plan_thresh={args.plan_thresh})</p>'
    if plan_mode_available else
    '<p style="color:#b71c1c">⚠ plan_fde mode not available — '
    're-run <code>lidar_step4_ray_casting.py</code> (now outputs planarity) '
    'then re-run this script to enable it.</p>'
)

rows_html = ''
for m in MODES:
    s = st[m]
    bg = ' style="background:#e8f5e9"' if m==best else ''
    rows_html += f"""<tr{bg}><td>{labels_m[m]}</td>
    <td>{s['n']}</td><td>{s['mean']:.2f}m</td><td>{s['rms']:.2f}m</td>
    <td>{s['p50']:.2f}m</td><td>{s['p95']:.2f}m</td>
    <td>{'—' if m=='baseline' else pct_change(m)}</td></tr>"""

html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Step 9a — FDE + Planarity SPP</title>
<style>
body{{font-family:sans-serif;margin:20px;background:#f4f6fb;color:#333}}
h1{{color:#1a237e;border-bottom:3px solid #4CAF50;padding-bottom:8px}}
h2{{color:#283593;margin-top:26px}}
img{{max-width:100%;border:1px solid #ddd;border-radius:6px;margin:8px 0}}
table{{border-collapse:collapse;width:100%}}
th{{background:#3f51b5;color:white;padding:8px 14px;text-align:left}}
td{{padding:7px 14px;border-bottom:1px solid #e8e8e8}}
tr:nth-child(even){{background:#f5f7ff}}
.better{{color:#2e7d32;font-weight:bold}}
.worse{{color:#c62828;font-weight:bold}}
.key{{background:#e8f5e9;padding:14px;border-left:4px solid #43a047;margin:12px 0;border-radius:4px}}
.note{{background:#e3f2fd;padding:12px;border-left:4px solid #1976d2;margin:10px 0;font-size:.93em;border-radius:4px}}
</style></head><body>
<h1>Step 9a — FDE/RAIM + Planarity-Weighted ΔL</h1>
<p>UrbanNav HK Medium-Urban-1 &nbsp;|&nbsp; {len(results)} epochs &nbsp;|&nbsp;
DD thresh={args.dd_thresh}m &nbsp;|&nbsp;
FDE: σ×{args.fde_sigma}, max {args.fde_max_excl} excl/epoch</p>
{plan_note}
<div class="key">
<table style="width:auto;margin-top:8px">
<tr><th>Mode</th><th>N</th><th>Mean</th><th>RMS</th><th>50th</th><th>95th</th><th>vs baseline</th></tr>
{rows_html}
</table>
Best mode: <b>{best}</b>
</div>
<h2>1. CDF</h2><img src="data:image/png;base64,{imgs['cdf']}">
<h2>2. Error by Mode</h2><img src="data:image/png;base64,{imgs['bar']}">
<h2>3. Time Series</h2><img src="data:image/png;base64,{imgs['ts']}">
<h2>4. Per-Epoch Improvement (DD+FDE vs Baseline)</h2>
<img src="data:image/png;base64,{imgs['improvement']}">
</body></html>"""

with open(args.out_html,'w') as f: f.write(html)
print(f'Saved HTML: {args.out_html}')
