#!/usr/bin/env python3
"""
lidar_step8b_fused_correct_spp.py
Fused C/N0 + DD-residual SPP correction.

Modes
-----
baseline   — elevation-weighted WLS, no correction (reference)
dd_corr    — GPS DD-NLOS: psr -= dd_resid  (Step 7e best result, carried forward)
cn0_dw     — GPS CN0-only-NLOS: weight *= cn0_dw  (isolated CN0 contribution)
fused      — GPS DD-NLOS: psr -= dd_resid
             GPS CN0-only-NLOS: weight *= cn0_dw  (combined approach)

Coverage
--------
dd_corr  covers  ~14 % of GPS obs (DD-flagged, t=5m)
fused    covers  ~30 % of GPS obs (CN0∪DD, precision=0.75 vs LiDAR)

Usage (inside container)
------------------------
python3 lidar_step8b_fused_correct_spp.py \\
  --obs        /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
  --nav_gps    /root/urbannav_gnss/hksc137c.21n \\
  --nav_bds    /root/urbannav_gnss/hksc137c.21f \\
  --nav_gal    /root/urbannav_gnss/hksc137c.21l \\
  --dd_labels  /root/dd_nlos_labels.csv \\
  --cn0_labels /root/cn0_nlos_labels.csv \\
  --gt         /root/urbannav_gt.txt \\
  --out_csv    /root/spp_fused_results.csv \\
  --out_html   /root/spp_fused_report.html
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
ap.add_argument('--obs',        default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps',    default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds',    default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal',    default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--dd_labels',  default='/root/dd_nlos_labels.csv')
ap.add_argument('--cn0_labels', default='/root/cn0_nlos_labels.csv')
ap.add_argument('--gt',         default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',    default='/root/spp_fused_results.csv')
ap.add_argument('--out_html',   default='/root/spp_fused_report.html')
ap.add_argument('--min_elev',   type=float, default=10.0)
ap.add_argument('--gt_tol',     type=float, default=10.0)
ap.add_argument('--dd_thresh',  type=float, default=5.0,
                help='|dd_resid| threshold (m) to flag DD-NLOS; Step 7e found '
                     '5m optimal (the nlos_dd column in the file is fixed at 30m)')
ap.add_argument('--cn0_dw',     type=float, default=0.3,
                help='weight factor for CN0-only NLOS satellites (default 0.3)')
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

# ── WLS SPP solver (multi-constellation) ─────────────────────────────────
def wls_spp_multi(sats_info, x0_ecef, min_sats=4, max_iter=10):
    active = [s for s in sats_info if s['weight'] > 0]
    if not active:
        return None, None, None
    sys_present = sorted(set(s['sys'] for s in active))
    n_clk = len(sys_present)
    if len(active) < max(min_sats, 3 + n_clk):
        return None, None, None
    clk_col = {sys: 3+i for i, sys in enumerate(sys_present)}
    n_unk = 3 + n_clk
    x = np.zeros(n_unk); x[:3] = x0_ecef

    H = None
    for _ in range(max_iter):
        H_list, dp_list, w_list = [], [], []
        for s in active:
            diff = x[:3] - np.array(s['sat_ecef'])
            r = np.linalg.norm(diff)
            if r < 1e4:
                continue
            e = diff / r
            row = np.zeros(n_unk); row[:3] = e
            row[clk_col[s['sys']]] = 1.0
            H_list.append(row)
            dp_list.append(s['psr_corr'] - r - x[clk_col[s['sys']]])
            w_list.append(s['weight'])
        if len(H_list) < max(min_sats, 3 + n_clk):
            return None, None, None
        H = np.array(H_list); dp = np.array(dp_list); W = np.diag(w_list)
        HtW = H.T @ W
        try:
            delta = np.linalg.solve(HtW @ H, HtW @ dp)
        except np.linalg.LinAlgError:
            return None, None, None
        x += delta
        if np.linalg.norm(delta[:3]) < 0.01:
            break

    try:
        Q = np.linalg.inv(H.T @ H)
        pdop = math.sqrt(max(Q[0,0]+Q[1,1]+Q[2,2], 0))
    except Exception:
        pdop = float('nan')

    return x[:3], {sys: float(x[col]) for sys, col in clk_col.items()}, pdop

# ── load DD labels (with dd_resid value, GPS only) ────────────────────────
print(f'Loading DD labels: {args.dd_labels}')
dd_map = {}   # (rinex_t_key, sat_id) -> {'nlos_dd':int, 'dd_resid':float}
with open(args.dd_labels) as f:
    for row in csv.DictReader(f):
        if row['sys'] != 'G':
            continue
        t_key = round(float(row['rinex_t']), 3)
        dd_map[(t_key, row['sat_id'])] = {
            'nlos_dd':  int(row['nlos_dd']),
            'dd_resid': float(row['dd_resid']),
        }
n_dd_nlos = sum(1 for v in dd_map.values() if v['nlos_dd'])
print(f'  {len(dd_map)} GPS DD records  ({n_dd_nlos} flagged NLOS)')

# ── load CN0 labels (GPS only) ────────────────────────────────────────────
print(f'Loading CN0 labels: {args.cn0_labels}')
cn0_map = {}   # (rinex_t_key, sat_id) -> {'nlos_cn0':int}
with open(args.cn0_labels) as f:
    for row in csv.DictReader(f):
        t_key = round(float(row['rinex_t']), 3)
        cn0_map[(t_key, row['sat_id'])] = {'nlos_cn0': int(row['nlos_cn0'])}
n_cn0_nlos = sum(1 for v in cn0_map.values() if v['nlos_cn0'])
print(f'  {len(cn0_map)} CN0 records  ({n_cn0_nlos} flagged NLOS)')

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
gt_times, gt_lats, gt_lons, gt_alts = (
    gt_times[order], gt_lats[order], gt_lons[order], gt_alts[order])
print(f'  {len(gt_times)} GT points')

def match_gt(utc_t):
    idx = int(np.searchsorted(gt_times, utc_t))
    idx = min(max(idx, 0), len(gt_times)-1)
    if idx > 0 and abs(gt_times[idx-1]-utc_t) < abs(gt_times[idx]-utc_t):
        idx -= 1
    return (gt_lats[idx], gt_lons[idx], gt_alts[idx]) \
           if abs(gt_times[idx]-utc_t) <= args.gt_tol else None

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

MODES = ('baseline', 'dd_corr', 'cn0_dw', 'fused')

# ── main loop ──────────────────────────────────────────────────────────────
print(f'\nRunning SPP ({len(MODES)} modes, min_elev={args.min_elev}°, '
      f'dd_thresh={args.dd_thresh}m, cn0_dw={args.cn0_dw})...')
results   = []
no_gt     = 0

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
        if sys_char not in ('G', 'C', 'E'):
            continue
        eph = find_closest_ephem(ephem.get(sat_id, []), rinex_t)
        if eph is None:
            continue
        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_t)
        if sat_ecef_raw is None:
            continue

        psr_raw = 0.0
        for k in PSR_KEYS.get(sys_char, ['C1C']):
            if k in obs.pseudorange and obs.pseudorange[k] > 0:
                psr_raw = obs.pseudorange[k]; break
        if psr_raw <= 0:
            continue

        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), psr_raw/C_LIGHT)
        diff_rx = sat_ecef_s - x0_ecef
        r_approx = np.linalg.norm(diff_rx)
        lat_r, lon_r = math.radians(gt_lat), math.radians(gt_lon)
        up = np.array([math.cos(lat_r)*math.cos(lon_r),
                       math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)])
        elev = math.degrees(math.asin(np.clip(np.dot(diff_rx/r_approx, up), -1, 1)))
        if elev < args.min_elev:
            continue

        psr_corr = psr_raw + C_LIGHT*dt_sv - tropo_delay(elev)
        w_base   = math.sin(math.radians(elev))**2

        # GPS NLOS flags (only trust GPS DD; CN0 available for all GPS obs)
        dd_info  = dd_map.get((rinex_t_key, sat_id))
        cn0_info = cn0_map.get((rinex_t_key, sat_id))

        dd_resid = float(dd_info['dd_resid']) if (dd_info  and sys_char == 'G') else 0.0
        # Flag DD-NLOS directly from |dd_resid| (Step 7e: 5m beats the file's 30m column)
        nlos_dd  = 1 if (dd_info and sys_char == 'G'
                         and abs(dd_resid) > args.dd_thresh) else 0
        nlos_cn0 = int(cn0_info['nlos_cn0']) if (cn0_info and sys_char == 'G') else 0

        # cn0-only: C/N0 says NLOS but DD does NOT (so no residual available to correct)
        nlos_cn0_only = 1 if (nlos_cn0 and not nlos_dd) else 0

        sat_data.append({
            'sat_id':       sat_id,
            'sys':          sys_char,
            'sat_ecef':     sat_ecef_s,
            'psr_corr':     psr_corr,
            'elev':         elev,
            'w_base':       w_base,
            'nlos_dd':      nlos_dd,
            'dd_resid':     dd_resid,
            'nlos_cn0_only': nlos_cn0_only,
        })

    if len(sat_data) < 4:
        continue

    mode_results = {}
    for mode in MODES:
        sats_info = []
        for s in sat_data:
            psr = s['psr_corr']
            w   = s['w_base']

            if mode == 'dd_corr':
                if s['nlos_dd']:
                    psr = s['psr_corr'] - s['dd_resid']

            elif mode == 'cn0_dw':
                if s['nlos_cn0_only']:
                    w = s['w_base'] * args.cn0_dw

            elif mode == 'fused':
                if s['nlos_dd']:
                    psr = s['psr_corr'] - s['dd_resid']
                elif s['nlos_cn0_only']:
                    w = s['w_base'] * args.cn0_dw
                # baseline: leave as-is

            sats_info.append({
                'sat_ecef': s['sat_ecef'],
                'psr_corr': psr,
                'weight':   w,
                'sys':      s['sys'],
            })

        pos, _, pdop = wls_spp_multi(sats_info, x0_ecef)
        if pos is None:
            mode_results[mode] = None; continue

        enu   = ecef_to_enu(pos, gt_ecef, gt_lat, gt_lon)
        err_h = math.sqrt(enu[0]**2 + enu[1]**2)
        err_v = abs(enu[2])
        mode_results[mode] = {
            'err_h':  round(err_h, 2),
            'err_v':  round(err_v, 2),
            'n_used': sum(1 for si in sats_info if si['weight'] > 0),
            'pdop':   round(pdop, 2) if not math.isnan(pdop) else None,
        }

    if all(v is None for v in mode_results.values()):
        continue

    n_dd_ep  = sum(1 for s in sat_data if s['nlos_dd'])
    n_cn0_ep = sum(1 for s in sat_data if s['nlos_cn0_only'])
    results.append({
        'utc_t':    round(utc_t, 3),
        'n_total':  len(sat_data),
        'n_dd':     n_dd_ep,
        'n_cn0only': n_cn0_ep,
        **{f'{m}_{k}': (mode_results[m][k] if mode_results[m] else None)
           for m in MODES for k in ('err_h', 'err_v', 'n_used', 'pdop')},
    })

    if (ep_i+1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} epochs, {len(results)} valid...',
              end='', flush=True)

print(f'\nDone: {len(results)} valid epochs  (no_gt={no_gt})')

# ── statistics ─────────────────────────────────────────────────────────────
def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return dict(n=0, mean=float('nan'), rms=float('nan'),
                    p50=float('nan'), p95=float('nan'))
    a = np.array(v)
    return dict(n=len(v), mean=float(np.mean(a)),
                rms=float(np.sqrt(np.mean(a**2))),
                p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)))

st = {m: stats([r[f'{m}_err_h'] for r in results]) for m in MODES}

print('\n=== Horizontal Error Summary ===')
print(f'  {"Mode":<12}  {"n":>4}  {"mean":>7}  {"RMS":>7}  {"50th":>7}  {"95th":>7}  vs-baseline')
for m in MODES:
    s = st[m]
    diff_str = ''
    if m != 'baseline':
        d = s['mean'] - st['baseline']['mean']
        diff_str = f'  {d:+.2f}m'
    print(f'  {m:<12}  {s["n"]:4d}  {s["mean"]:7.2f}m  {s["rms"]:7.2f}m  '
          f'{s["p50"]:7.2f}m  {s["p95"]:7.2f}m{diff_str}')

avg_dd  = np.mean([r['n_dd']      for r in results])
avg_cn0 = np.mean([r['n_cn0only'] for r in results])
print(f'\n  Avg per epoch: DD-corrected={avg_dd:.1f} sats, '
      f'CN0-downweighted={avg_cn0:.1f} sats')

# ── save CSV ───────────────────────────────────────────────────────────────
COLS = ['utc_t', 'n_total', 'n_dd', 'n_cn0only'] + \
       [f'{m}_{k}' for m in MODES for k in ('err_h', 'err_v', 'n_used', 'pdop')]
with open(args.out_csv, 'w', newline='') as f:
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
    'cn0_dw':   '#FF9800',
    'fused':    '#4CAF50',
}
labels_m = {
    'baseline': 'Baseline',
    'dd_corr':  'DD-corr (14% cov)',
    'cn0_dw':   f'CN0-dw×{args.cn0_dw} (8% extra)',
    'fused':    f'Fused DD-corr+CN0-dw (30% cov)',
}

imgs = {}

# Fig 1: CDF
fig, ax = plt.subplots(figsize=(9, 5))
for m in MODES:
    vals = sorted(r[f'{m}_err_h'] for r in results if r[f'{m}_err_h'] is not None)
    if vals:
        lw = 2.8 if m == 'fused' else 1.6
        ax.plot(vals, np.linspace(0, 1, len(vals)),
                color=colors[m], lw=lw,
                label=f'{labels_m[m]}  mean={st[m]["mean"]:.2f}m')
ax.set_xlabel('Horizontal error (m)'); ax.set_ylabel('CDF')
ax.set_title('SPP Horizontal Error CDF — Fused C/N0 + DD Correction')
ax.set_xlim(0, 200); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
imgs['cdf'] = fig_to_b64(fig)

# Fig 2: bar — all 4 metrics
fig, ax = plt.subplots(figsize=(10, 5))
metrics  = ['mean', 'rms', 'p50', 'p95']
mlabels  = ['Mean', 'RMS', '50th pct', '95th pct']
x = np.arange(len(metrics)); bw = 0.2
for i, m in enumerate(MODES):
    vals = [st[m][k] for k in metrics]
    ax.bar(x + i*bw, vals, bw, label=labels_m[m], color=colors[m], alpha=0.85,
           edgecolor='white')
ax.set_xticks(x + 1.5*bw); ax.set_xticklabels(mlabels)
ax.set_ylabel('Error (m)'); ax.set_title('SPP Horizontal Error by Mode')
ax.legend(fontsize=9); ax.grid(axis='y', alpha=0.3)
imgs['bar'] = fig_to_b64(fig)

# Fig 3: time series
fig, ax = plt.subplots(figsize=(13, 4.5))
for m in MODES:
    ts = [(r['utc_t'], r[f'{m}_err_h']) for r in results if r[f'{m}_err_h'] is not None]
    if ts:
        lw = 2.2 if m == 'fused' else 1.0
        ax.plot([t for t,_ in ts], [e for _,e in ts],
                color=colors[m], lw=lw, alpha=0.8, label=labels_m[m])
ax.set_xlabel('UTC time (s)'); ax.set_ylabel('Horizontal error (m)')
ax.set_title('SPP Horizontal Error — Time Series')
ax.legend(fontsize=9); ax.grid(True, alpha=0.3); ax.set_ylim(0, 300)
imgs['ts'] = fig_to_b64(fig)

# Fig 4: per-epoch improvement (fused vs baseline)
fig, ax = plt.subplots(figsize=(13, 4))
paired = [(r['utc_t'], r['baseline_err_h'] - r['fused_err_h'])
          for r in results
          if r['baseline_err_h'] is not None and r['fused_err_h'] is not None]
if paired:
    ts_p = [t for t,_ in paired]
    impr = [d for _,d in paired]
    pos_frac = 100 * sum(1 for d in impr if d > 0) / len(impr)
    ax.bar(ts_p, impr, width=2,
           color=['#4CAF50' if d > 0 else '#F44336' for d in impr], alpha=0.75)
    ax.axhline(0, color='k', lw=0.8)
    ax.set_xlabel('UTC time (s)')
    ax.set_ylabel('Improvement (m)\n(positive = fused better)')
    ax.set_title(f'Per-epoch improvement: Fused vs Baseline  '
                 f'(fused better in {pos_frac:.0f}% of epochs)')
    ax.grid(axis='y', alpha=0.3)
imgs['improvement'] = fig_to_b64(fig)

# ── HTML ───────────────────────────────────────────────────────────────────
def pct_change(m):
    d = st[m]['mean'] - st['baseline']['mean']
    arr = '&darr;' if d < 0 else '&uarr;'
    cls = 'better' if d < 0 else 'worse'
    return f'<span class="{cls}">{arr}{abs(d):.2f}m</span>'

best_mode = min(MODES, key=lambda m: st[m]['mean'])

html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Step 8b — Fused C/N0 + DD SPP Correction</title>
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
<h1>Step 8b — Fused C/N0 + DD-residual SPP Correction</h1>
<p>UrbanNav HK Medium-Urban-1 &nbsp;|&nbsp; {len(results)} valid epochs &nbsp;|&nbsp;
Elevation cutoff: {args.min_elev}&deg; &nbsp;|&nbsp; CN0 down-weight: &times;{args.cn0_dw}</p>

<div class="note">
<b>Strategy:</b>
<ul style="margin:6px 0">
<li><b>DD-flagged GPS sats</b> (|dd_resid|&gt;5m, t=5m): apply pseudorange correction
    <code>psr &minus;= dd_resid</code> &mdash; removes NLOS path excess directly.</li>
<li><b>CN0-only GPS sats</b> (C/N0 deficit&gt;6dB, DD not flagged): down-weight by
    &times;{args.cn0_dw} &mdash; no residual available, reduce influence without
    destroying geometric diversity.</li>
<li><b>All other sats</b>: standard elevation-weighted WLS (unchanged).</li>
</ul>
Combined coverage: ~30% of GPS obs vs ~14% for DD alone.
</div>

<div class="key">
<b>Horizontal error summary</b><br>
<table style="width:auto;margin-top:10px">
<tr><th>Mode</th><th>N</th><th>Mean</th><th>RMS</th><th>50th</th><th>95th</th><th>vs baseline</th></tr>
<tr><td>Baseline</td>
    <td>{st['baseline']['n']}</td>
    <td>{st['baseline']['mean']:.2f}m</td>
    <td>{st['baseline']['rms']:.2f}m</td>
    <td>{st['baseline']['p50']:.2f}m</td>
    <td>{st['baseline']['p95']:.2f}m</td>
    <td>—</td></tr>
<tr><td>DD-corr (14% cov)</td>
    <td>{st['dd_corr']['n']}</td>
    <td>{st['dd_corr']['mean']:.2f}m</td>
    <td>{st['dd_corr']['rms']:.2f}m</td>
    <td>{st['dd_corr']['p50']:.2f}m</td>
    <td>{st['dd_corr']['p95']:.2f}m</td>
    <td>{pct_change('dd_corr')}</td></tr>
<tr><td>CN0-dw&times;{args.cn0_dw} (8% extra)</td>
    <td>{st['cn0_dw']['n']}</td>
    <td>{st['cn0_dw']['mean']:.2f}m</td>
    <td>{st['cn0_dw']['rms']:.2f}m</td>
    <td>{st['cn0_dw']['p50']:.2f}m</td>
    <td>{st['cn0_dw']['p95']:.2f}m</td>
    <td>{pct_change('cn0_dw')}</td></tr>
<tr style="background:#e8f5e9"><td><b>Fused (30% cov)</b></td>
    <td>{st['fused']['n']}</td>
    <td><b>{st['fused']['mean']:.2f}m</b></td>
    <td>{st['fused']['rms']:.2f}m</td>
    <td>{st['fused']['p50']:.2f}m</td>
    <td>{st['fused']['p95']:.2f}m</td>
    <td><b>{pct_change('fused')}</b></td></tr>
</table>
<br>Best mode: <b>{best_mode}</b>
</div>

<h2>1. CDF — Horizontal Error</h2>
<img src="data:image/png;base64,{imgs['cdf']}">

<h2>2. Error Statistics by Mode</h2>
<img src="data:image/png;base64,{imgs['bar']}">

<h2>3. Time Series</h2>
<img src="data:image/png;base64,{imgs['ts']}">

<h2>4. Per-Epoch Improvement (Fused vs Baseline)</h2>
<img src="data:image/png;base64,{imgs['improvement']}">

</body></html>"""

with open(args.out_html, 'w') as f:
    f.write(html)
print(f'Saved HTML: {args.out_html}')
