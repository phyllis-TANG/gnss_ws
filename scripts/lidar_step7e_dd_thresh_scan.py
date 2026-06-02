#!/usr/bin/env python3
"""
lidar_step7e_dd_thresh_scan.py
Scan DD-residual correction thresholds to find the optimal value.

The dd_nlos_labels.csv contains dd_resid for every GPS observation.
For each threshold t, satellites with |dd_resid| > t get psr -= dd_resid.
No need to re-run Step 6b — threshold is purely a post-processing parameter.

Trade-off intuition:
  t too high  -> few corrections, low coverage, mean stays near baseline
  t too low   -> LOS sats get "corrected" by noise -> degrades solution
  optimal t   -> enough NLOS caught, noise corrections small enough to ignore

Usage (inside container):
  python3 lidar_step7e_dd_thresh_scan.py \
    --obs       /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
    --nav_gps   /root/urbannav_gnss/hksc137c.21n \
    --nav_bds   /root/urbannav_gnss/hksc137c.21f \
    --nav_gal   /root/urbannav_gnss/hksc137c.21l \
    --dd_labels /root/dd_nlos_labels.csv \
    --gt        /root/urbannav_gt.txt \
    --out_html  /root/dd_thresh_scan_report.html
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
                          llh_to_ecef)

LEAP_SECONDS = 18
C_LIGHT      = 299792458.0
OMEGA_E      = 7.2921151467e-5

THRESHOLDS = [5, 10, 15, 20, 25, 30, 40, 50, 75, 100]

PSR_KEYS = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
}

ap = argparse.ArgumentParser()
ap.add_argument('--obs',       default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps',   default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds',   default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal',   default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--dd_labels', default='/root/dd_nlos_labels.csv')
ap.add_argument('--gt',        default='/root/urbannav_gt.txt')
ap.add_argument('--out_html',  default='/root/dd_thresh_scan_report.html')
ap.add_argument('--out_csv',   default='/root/dd_thresh_scan_results.csv')
ap.add_argument('--min_elev',  type=float, default=10.0)
ap.add_argument('--gt_tol',    type=float, default=10.0)
args = ap.parse_args()

# ── helpers ───────────────────────────────────────────────────────────
def load_nav(path, prefix):
    try:
        raw = read_rinex_nav(path)
    except Exception as e:
        print(f'  [WARN] {path}: {e}'); return {}
    out = {}
    for k, v in raw.items():
        if isinstance(k, int):   nk = f'{prefix}{k:02d}'
        elif isinstance(k, str) and k and k[0] != prefix: nk = prefix + k[1:]
        else: nk = k
        out[nk] = v
    return out

def sagnac_correct(sat_ecef, travel_time_s):
    theta = OMEGA_E * travel_time_s
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ sat_ecef

def tropo_delay(elev_deg):
    return 2.3 / math.sin(math.radians(max(elev_deg, 3.0)) + 0.017)

def ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon):
    lat, lon = math.radians(ref_lat), math.radians(ref_lon)
    R = np.array([[-math.sin(lon), math.cos(lon), 0],
                  [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
                  [ math.cos(lat)*math.cos(lon),  math.cos(lat)*math.sin(lon), math.sin(lat)]])
    return R @ (ecef - ref_ecef)

def dms_to_deg(d, m, s):
    return float(d) + float(m)/60 + float(s)/3600

def wls_spp(sats_info, x0_ecef, max_iter=10):
    active = [s for s in sats_info if s['w'] > 0]
    sys_present = sorted(set(s['sys'] for s in active))
    n_clk = len(sys_present)
    if len(active) < max(4, 3 + n_clk):
        return None
    clk_col = {sys: 3+i for i, sys in enumerate(sys_present)}
    n_unk = 3 + n_clk
    x = np.zeros(n_unk); x[:3] = x0_ecef

    for _ in range(max_iter):
        H_list, dp_list, w_list = [], [], []
        for s in active:
            diff = x[:3] - np.array(s['sat_ecef'])
            r = np.linalg.norm(diff)
            if r < 1e4: continue
            row = np.zeros(n_unk); row[:3] = diff/r
            row[clk_col[s['sys']]] = 1.0
            H_list.append(row)
            dp_list.append(s['psr'] - r - x[clk_col[s['sys']]])
            w_list.append(s['w'])
        if len(H_list) < max(4, 3 + n_clk):
            return None
        H = np.array(H_list); dp = np.array(dp_list); W = np.diag(w_list)
        HtW = H.T @ W
        try:
            delta = np.linalg.solve(HtW @ H, HtW @ dp)
        except np.linalg.LinAlgError:
            return None
        x += delta
        if np.linalg.norm(delta[:3]) < 0.01: break

    return x[:3]

# ── load GT ───────────────────────────────────────────────────────────
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
            gt_times.append(t); gt_lats.append(dms_to_deg(p[3],p[4],p[5]))
            gt_lons.append(dms_to_deg(p[6],p[7],p[8])); gt_alts.append(float(p[9]))
        except (ValueError, IndexError): continue

gt_times = np.array(gt_times); gt_lats = np.array(gt_lats)
gt_lons  = np.array(gt_lons);  gt_alts  = np.array(gt_alts)
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

# ── load DD labels ────────────────────────────────────────────────────
print(f'Loading DD labels: {args.dd_labels}')
dd_table = {}   # (rinex_t_key, sat_id) -> dd_resid  (GPS only)
with open(args.dd_labels) as f:
    for row in csv.DictReader(f):
        if row['sys'] != 'G': continue
        key = (round(float(row['rinex_t']), 3), row['sat_id'])
        dd_table[key] = float(row['dd_resid'])
print(f'  {len(dd_table)} GPS DD records loaded')

# ── load nav + obs ────────────────────────────────────────────────────
print('Loading navigation...')
ephem = {}
ephem.update(load_nav(args.nav_gps, 'G'))
ephem.update(load_nav(args.nav_bds, 'C'))
ephem.update(load_nav(args.nav_gal, 'E'))
print(f'  {sum(len(v) for v in ephem.values())} ephemerides')

print(f'Loading obs: {args.obs}')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} epochs')

x0_ecef = np.array(llh_to_ecef(22.3198, 114.2095, 20.0))

# ── pre-compute per-epoch satellite data (done once, reused per threshold) ──
print('\nPre-computing satellite data...')
epoch_cache = []   # list of {utc_t, gt_ecef, gt_lat, gt_lon, sats:[...]}

for epoch in obs_epochs:
    rinex_t     = epoch.time_unix
    utc_t       = rinex_t - LEAP_SECONDS
    rinex_t_key = round(rinex_t, 3)

    gt_match = match_gt(utc_t)
    if gt_match is None: continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    sats = []
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
        dd_resid = dd_table.get((rinex_t_key, sat_id), None)

        sats.append({
            'sat_ecef': sat_ecef_s, 'psr_corr': psr_corr,
            'w_base': w_base, 'sys': sys_char,
            'dd_resid': dd_resid,   # None if not in GPS dd_table
        })

    if len(sats) >= 4:
        epoch_cache.append({
            'utc_t': utc_t, 'gt_ecef': gt_ecef,
            'gt_lat': gt_lat, 'gt_lon': gt_lon, 'sats': sats,
        })

print(f'  {len(epoch_cache)} usable epochs cached')

# ── run SPP for each threshold ────────────────────────────────────────
print(f'\nScanning thresholds: {THRESHOLDS} m ...')

# baseline first
baseline_errs = []
for ec in epoch_cache:
    sats_info = [{'sat_ecef': s['sat_ecef'], 'psr': s['psr_corr'],
                  'w': s['w_base'], 'sys': s['sys']} for s in ec['sats']]
    pos = wls_spp(sats_info, x0_ecef)
    if pos is None: continue
    enu = ecef_to_enu(pos, ec['gt_ecef'], ec['gt_lat'], ec['gt_lon'])
    baseline_errs.append(math.sqrt(enu[0]**2 + enu[1]**2))

def err_stats(errs):
    if not errs: return dict(n=0, mean=float('nan'), rms=float('nan'),
                              p50=float('nan'), p95=float('nan'))
    a = np.array(errs)
    return dict(n=len(a), mean=float(np.mean(a)),
                rms=float(np.sqrt(np.mean(a**2))),
                p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)))

baseline_st = err_stats(baseline_errs)
print(f'  baseline: mean={baseline_st["mean"]:.2f}m  RMS={baseline_st["rms"]:.2f}m  '
      f'p95={baseline_st["p95"]:.2f}m  n={baseline_st["n"]}')

scan_results = []        # one row per threshold
all_errs     = {}        # thresh -> list of err_h (for CDF plots)

for t in THRESHOLDS:
    errs = []; n_corrected = 0
    for ec in epoch_cache:
        sats_info = []
        for s in ec['sats']:
            psr = s['psr_corr']; w = s['w_base']
            dr  = s['dd_resid']
            if dr is not None and abs(dr) > t:
                psr = s['psr_corr'] - dr
                n_corrected += 1
            sats_info.append({'sat_ecef': s['sat_ecef'], 'psr': psr,
                               'w': w, 'sys': s['sys']})
        pos = wls_spp(sats_info, x0_ecef)
        if pos is None: continue
        enu = ecef_to_enu(pos, ec['gt_ecef'], ec['gt_lat'], ec['gt_lon'])
        errs.append(math.sqrt(enu[0]**2 + enu[1]**2))

    st = err_stats(errs)
    coverage = n_corrected / max(len(epoch_cache), 1)
    scan_results.append({
        'thresh': t, 'n_corrected': n_corrected,
        'corr_per_epoch': round(n_corrected / max(len(epoch_cache),1), 2),
        **{k: round(v, 3) for k, v in st.items()},
        'delta_mean': round(st['mean'] - baseline_st['mean'], 3),
        'delta_rms':  round(st['rms']  - baseline_st['rms'],  3),
        'delta_p95':  round(st['p95']  - baseline_st['p95'],  3),
    })
    all_errs[t] = errs
    print(f'  t={t:3d}m  corrected={n_corrected:4d} ({coverage:.1f}/ep)  '
          f'mean={st["mean"]:.2f}m  RMS={st["rms"]:.2f}m  p95={st["p95"]:.2f}m  '
          f'Δmean={st["mean"]-baseline_st["mean"]:+.2f}m')

# ── find optimal threshold ────────────────────────────────────────────
best_mean = min(scan_results, key=lambda r: r['mean'])
best_rms  = min(scan_results, key=lambda r: r['rms'])
best_p95  = min(scan_results, key=lambda r: r['p95'])

print(f'\nBest by mean: t={best_mean["thresh"]}m  mean={best_mean["mean"]:.2f}m '
      f'({best_mean["delta_mean"]:+.2f}m vs baseline)')
print(f'Best by RMS:  t={best_rms["thresh"]}m   RMS={best_rms["rms"]:.2f}m '
      f'({best_rms["delta_rms"]:+.2f}m vs baseline)')
print(f'Best by p95:  t={best_p95["thresh"]}m   p95={best_p95["p95"]:.2f}m '
      f'({best_p95["delta_p95"]:+.2f}m vs baseline)')

# ── save CSV ──────────────────────────────────────────────────────────
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(scan_results[0].keys()))
    w.writeheader(); w.writerows(scan_results)
print(f'Saved: {args.out_csv}')

# ── plots ─────────────────────────────────────────────────────────────
def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

thresholds  = [r['thresh']     for r in scan_results]
means       = [r['mean']       for r in scan_results]
rmss        = [r['rms']        for r in scan_results]
p95s        = [r['p95']        for r in scan_results]
corr_per_ep = [r['corr_per_epoch'] for r in scan_results]

# Fig 1: trade-off curve
fig, ax1 = plt.subplots(figsize=(10, 5))
ax2 = ax1.twinx()
ax1.plot(thresholds, means, 'o-', color='#2196F3', lw=2, label='Mean error')
ax1.plot(thresholds, rmss,  's-', color='#9C27B0', lw=2, label='RMS error')
ax1.plot(thresholds, p95s,  '^-', color='#FF5722', lw=2, label='95th pct')
ax1.axhline(baseline_st['mean'], color='#2196F3', ls='--', lw=1, alpha=0.5, label='Baseline mean')
ax1.axhline(baseline_st['rms'],  color='#9C27B0', ls='--', lw=1, alpha=0.5, label='Baseline RMS')
ax1.axhline(baseline_st['p95'],  color='#FF5722', ls='--', lw=1, alpha=0.5, label='Baseline p95')
ax1.axvline(best_mean['thresh'], color='#2196F3', ls=':', lw=1.5,
            label=f'Best mean t={best_mean["thresh"]}m')
ax2.bar(thresholds, corr_per_ep, width=3, color='#4CAF50', alpha=0.25,
        label='Corrections/epoch')
ax1.set_xlabel('DD threshold (m)'); ax1.set_ylabel('Horizontal error (m)')
ax2.set_ylabel('Corrections per epoch', color='#4CAF50')
ax1.set_title('DD-residual Correction: Threshold vs Accuracy Trade-off')
lines1, lab1 = ax1.get_legend_handles_labels()
lines2, lab2 = ax2.get_legend_handles_labels()
ax1.legend(lines1+lines2, lab1+lab2, loc='upper right', fontsize=8)
ax1.grid(True, alpha=0.3)
img_tradeoff = fig_to_b64(fig)

# Fig 2: delta-metrics vs threshold
fig, ax = plt.subplots(figsize=(10, 4))
delta_means = [r['delta_mean'] for r in scan_results]
delta_rmss  = [r['delta_rms']  for r in scan_results]
delta_p95s  = [r['delta_p95']  for r in scan_results]
ax.plot(thresholds, delta_means, 'o-', color='#2196F3', lw=2, label='Δmean')
ax.plot(thresholds, delta_rmss,  's-', color='#9C27B0', lw=2, label='ΔRMS')
ax.plot(thresholds, delta_p95s,  '^-', color='#FF5722', lw=2, label='Δp95')
ax.axhline(0, color='k', lw=1, ls='--')
ax.fill_between(thresholds, delta_means, 0,
                where=[d < 0 for d in delta_means],
                alpha=0.15, color='#2196F3')
ax.axvline(best_mean['thresh'], color='#2196F3', ls=':', lw=1.5,
           label=f'Best mean t={best_mean["thresh"]}m')
ax.set_xlabel('DD threshold (m)'); ax.set_ylabel('Error change vs baseline (m)')
ax.set_title('Improvement over Baseline by Threshold  (negative = better)')
ax.legend(); ax.grid(True, alpha=0.3)
img_delta = fig_to_b64(fig)

# Fig 3: CDF comparison
cdf_thresholds = sorted(set([5, 10, best_mean['thresh'], 30, 100]))
cmap = plt.get_cmap('tab10')
fig, ax = plt.subplots(figsize=(9, 5))
bl_sorted = sorted(baseline_errs)
ax.plot(bl_sorted, np.linspace(0, 1, len(bl_sorted)),
        color='k', lw=2, label=f'Baseline (mean={baseline_st["mean"]:.1f}m)')
for i, t in enumerate(cdf_thresholds):
    errs_t = sorted(all_errs[t])
    st_t   = err_stats(errs_t)
    lw = 2.5 if t == best_mean['thresh'] else 1.2
    marker = ' ★' if t == best_mean['thresh'] else ''
    ax.plot(errs_t, np.linspace(0, 1, len(errs_t)),
            color=cmap(i), lw=lw,
            label=f't={t}m (mean={st_t["mean"]:.1f}m){marker}')
ax.set_xlabel('Horizontal error (m)'); ax.set_ylabel('CDF')
ax.set_title('SPP Horizontal Error CDF — DD Threshold Comparison')
ax.set_xlim(0, 200); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
img_cdf = fig_to_b64(fig)

# ── HTML ──────────────────────────────────────────────────────────────
def fmt_delta(v):
    if v < -0.05: return f'<span style="color:#2e7d32;font-weight:bold">{v:+.2f}m ▼</span>'
    if v >  0.05: return f'<span style="color:#c62828">{v:+.2f}m ▲</span>'
    return f'{v:+.2f}m'

rows_html = ''
for r in scan_results:
    best_mark  = ' ★' if r['thresh'] == best_mean['thresh'] else ''
    row_style  = '  style="background:#e8f5e9"' if r['thresh'] == best_mean['thresh'] else ''
    rows_html += (
        f'<tr{row_style}>'
        f'<td><b>{r["thresh"]}m{best_mark}</b></td>'
        f'<td>{r["n_corrected"]} ({r["corr_per_epoch"]:.1f}/ep)</td>'
        f'<td>{r["mean"]:.2f}</td><td>{r["rms"]:.2f}</td>'
        f'<td>{r["p50"]:.2f}</td><td>{r["p95"]:.2f}</td>'
        f'<td>{fmt_delta(r["delta_mean"])}</td>'
        f'<td>{fmt_delta(r["delta_rms"])}</td>'
        f'<td>{fmt_delta(r["delta_p95"])}</td></tr>\n'
    )

html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Step 7e — DD Threshold Scan</title>
<style>
body{{font-family:sans-serif;margin:20px;background:#f4f6fb;color:#333}}
h1{{color:#1a237e;border-bottom:3px solid #3f51b5;padding-bottom:8px}}
h2{{color:#283593;margin-top:26px}}
img{{max-width:100%;border:1px solid #ddd;border-radius:6px;margin:8px 0}}
table{{border-collapse:collapse;width:100%;font-size:.92em}}
th{{background:#3f51b5;color:white;padding:7px 12px;text-align:left}}
td{{padding:6px 12px;border-bottom:1px solid #e8e8e8}}
.key{{background:#e8f5e9;padding:14px;border-left:4px solid #43a047;margin:12px 0;border-radius:4px}}
.note{{background:#e3f2fd;padding:12px;border-left:4px solid #1976d2;margin:10px 0;font-size:.93em;border-radius:4px}}
</style></head><body>
<h1>Step 7e — DD-residual Threshold Scan</h1>
<p>UrbanNav HK Medium-Urban-1 &nbsp;|&nbsp; {len(epoch_cache)} valid epochs &nbsp;|&nbsp;
Elevation cutoff: {args.min_elev}&deg;</p>

<div class="note">
<b>Method:</b> For each threshold <i>t</i>, GPS satellites with |dd_resid|&nbsp;&gt;&nbsp;<i>t</i>
have their pseudorange corrected as <code>psr&nbsp;-=&nbsp;dd_resid</code>.
No other satellite is touched. Baseline uses no corrections.
</div>

<div class="key">
<b>Baseline:</b> mean={baseline_st['mean']:.2f}m &nbsp; RMS={baseline_st['rms']:.2f}m &nbsp; p95={baseline_st['p95']:.2f}m<br>
<b>Best by mean:</b> t=<b>{best_mean['thresh']}m</b> &rarr; mean={best_mean['mean']:.2f}m
  &nbsp; ({best_mean['delta_mean']:+.2f}m) &nbsp; RMS={best_mean['rms']:.2f}m
  &nbsp; p95={best_mean['p95']:.2f}m ({best_mean['delta_p95']:+.2f}m)<br>
<b>Best by RMS:</b> t={best_rms['thresh']}m &rarr; RMS={best_rms['rms']:.2f}m
  &nbsp; ({best_rms['delta_rms']:+.2f}m)<br>
<b>Best by p95:</b> t={best_p95['thresh']}m &rarr; p95={best_p95['p95']:.2f}m
  &nbsp; ({best_p95['delta_p95']:+.2f}m)
</div>

<table>
<tr><th>Threshold</th><th>Corrections</th>
    <th>Mean&nbsp;(m)</th><th>RMS&nbsp;(m)</th><th>50th&nbsp;(m)</th><th>95th&nbsp;(m)</th>
    <th>Δmean</th><th>ΔRMS</th><th>Δp95</th></tr>
<tr style="background:#fff3e0">
  <td><b>baseline</b></td><td>0</td>
  <td>{baseline_st['mean']:.2f}</td><td>{baseline_st['rms']:.2f}</td>
  <td>{baseline_st['p50']:.2f}</td><td>{baseline_st['p95']:.2f}</td>
  <td>—</td><td>—</td><td>—</td></tr>
{rows_html}
</table>

<h2>1. Accuracy vs Threshold (Trade-off Curve)</h2>
<img src="data:image/png;base64,{img_tradeoff}">

<h2>2. Improvement over Baseline by Threshold</h2>
<img src="data:image/png;base64,{img_delta}">

<h2>3. CDF Comparison (Baseline vs Selected Thresholds)</h2>
<img src="data:image/png;base64,{img_cdf}">

</body></html>"""

with open(args.out_html, 'w') as f:
    f.write(html)
print(f'Saved HTML: {args.out_html}')
