#!/usr/bin/env python3
"""
lidar_step8b_compare_bounces.py
Compare 1-bounce vs 2-bounce delta-L models against GT-derived actual
pseudorange errors.  Uses lidar_nlos_prediction_2b.csv (Step4 output)
as the NLOS source, replacing the old nlos + refl pair.

Usage (inside container):
  python3 lidar_step8b_compare_bounces.py \
    --obs     /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
    --nav_gps /root/urbannav_gnss/hksc137c.21n \
    --nav_bds /root/urbannav_gnss/hksc137c.21f \
    --nav_gal /root/urbannav_gnss/hksc137c.21l \
    --nlos_2b /root/lidar_nlos_prediction_2b.csv \
    --gt      /root/urbannav_gt.txt \
    --out_csv  /root/residual_2b.csv \
    --out_html /root/residual_2b.html
"""

import argparse, base64, csv, io, math, os, sys
import numpy as np
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── rinex_utils path ─────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in [
    '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts',
    '/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts',
    os.path.join(SCRIPT_DIR, 'src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts'),
]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rinex_utils import (read_rinex_obs, read_rinex_nav,
                          compute_sat_position, find_closest_ephem,
                          llh_to_ecef)

LEAP_SECONDS = 18
C_LIGHT      = 299792458.0
OMEGA_E      = 7.2921151467e-5

PSR_KEYS = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
}

ap = argparse.ArgumentParser()
ap.add_argument('--obs',      default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps',  default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds',  default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal',  default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--nlos_2b',  default='/root/lidar_nlos_prediction_2b.csv')
ap.add_argument('--gt',       default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',  default='/root/residual_2b.csv')
ap.add_argument('--out_html', default='/root/residual_2b.html')
ap.add_argument('--min_elev', type=float, default=10.0)
ap.add_argument('--gt_tol',   type=float, default=10.0)
ap.add_argument('--min_los',  type=int, default=3)
args = ap.parse_args()

# ── helpers ──────────────────────────────────────────────────────────
def sagnac_correct(sat_ecef, travel_time_s):
    theta = OMEGA_E * travel_time_s
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ sat_ecef

def tropo_delay(elev_deg):
    el = max(elev_deg, 3.0)
    return 2.3 / math.sin(math.radians(el) + 0.017)

def elev_from_ecef(rx_ecef, sat_ecef, rx_lat, rx_lon):
    diff = sat_ecef - rx_ecef
    r = np.linalg.norm(diff)
    lat, lon = math.radians(rx_lat), math.radians(rx_lon)
    up = np.array([math.cos(lat)*math.cos(lon),
                   math.cos(lat)*math.sin(lon),
                   math.sin(lat)])
    return math.degrees(math.asin(np.clip(np.dot(diff/r, up), -1, 1)))

def sat_dir_enu(azim_deg, elev_deg):
    az, el = math.radians(azim_deg), math.radians(elev_deg)
    return np.array([math.sin(az)*math.cos(el),
                     math.cos(az)*math.cos(el),
                     math.sin(el)])

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

def dms_to_deg(d, m, s):
    return float(d) + float(m)/60 + float(s)/3600

# ── load 2b NLOS data ────────────────────────────────────────────────
print(f'Loading 2b NLOS data: {args.nlos_2b}')
nlos2b = {}
with open(args.nlos_2b) as f:
    for row in csv.DictReader(f):
        if int(row['lidar_nlos']) == 0:
            continue
        # key: (unix_t_str, sat_id) — same format as step8's rinex_t key
        nlos2b[(row['unix_t'], row['sat_id'])] = {
            'n_b':    int(row['n_bounces']),
            'dist1':  float(row['hit_dist_m']),
            'dist2':  float(row['hit_dist2_m']),
            'ne':     float(row['normal_e']),
            'nn':     float(row['normal_n']),
            'nu':     float(row['normal_u']),
            'azim':   float(row['azimuth_deg']),
            'elev2b': float(row['elevation_deg']),
        }
print(f'  {len(nlos2b)} NLOS records')

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
            if t < 1e9:
                continue
            gt_times.append(t)
            gt_lats.append(dms_to_deg(p[3], p[4], p[5]))
            gt_lons.append(dms_to_deg(p[6], p[7], p[8]))
            gt_alts.append(float(p[9]))
        except (ValueError, IndexError):
            continue

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
    if abs(gt_times[idx]-utc_t) > args.gt_tol:
        return None
    return gt_lats[idx], gt_lons[idx], gt_alts[idx]

# ── load nav + obs ───────────────────────────────────────────────────
print('Loading navigation messages...')
ephem = {}
ephem.update(load_nav(args.nav_gps, 'G'))
ephem.update(load_nav(args.nav_bds, 'C'))
ephem.update(load_nav(args.nav_gal, 'E'))
print(f'  {sum(len(v) for v in ephem.values())} ephemerides for {len(ephem)} sats')

print(f'Loading obs: {args.obs}')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} epochs')

# ── main loop ────────────────────────────────────────────────────────
out_rows   = []
epoch_skip = clk_fail = 0
key_hits = key_miss = 0

for ep_i, epoch in enumerate(obs_epochs):
    rinex_t = epoch.time_unix
    utc_t   = rinex_t - LEAP_SECONDS

    gt_match = match_gt(utc_t)
    if gt_match is None:
        epoch_skip += 1
        continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    sats = []
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

        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), psr_raw / C_LIGHT)
        elev = elev_from_ecef(gt_ecef, sat_ecef_s, gt_lat, gt_lon)
        if elev < args.min_elev:
            continue

        psr_corr = psr_raw + C_LIGHT * dt_sv - tropo_delay(elev)
        r_gt     = float(np.linalg.norm(sat_ecef_s - gt_ecef))

        # look up 2b NLOS info — try rinex_t first, then utc_t
        key_r = (f'{rinex_t:.3f}', sat_id)
        key_u = (f'{utc_t:.3f}',   sat_id)
        info_2b = nlos2b.get(key_r) or nlos2b.get(key_u)

        if info_2b:
            key_hits += 1
        else:
            key_miss += 1

        sats.append({
            'sat_id':    sat_id,
            'sys':       sys_char,
            'elev':      elev,
            'psr_corr':  psr_corr,
            'r_gt':      r_gt,
            'is_nlos':   1 if info_2b else 0,
            'info_2b':   info_2b,
            'utc_t':     utc_t,
        })

    if not sats:
        continue

    # clock bias: median over LOS sats per constellation
    los_by_sys = defaultdict(list)
    for s in sats:
        if s['is_nlos'] == 0:
            los_by_sys[s['sys']].append(s['psr_corr'] - s['r_gt'])

    if not any(len(v) >= args.min_los for v in los_by_sys.values()):
        clk_fail += 1
        continue

    clk_bias = {sys: float(np.median(res))
                for sys, res in los_by_sys.items()
                if len(res) >= args.min_los}

    for s in sats:
        if s['is_nlos'] != 1:
            continue
        sys = s['sys']
        if sys not in clk_bias:
            continue

        actual_error = s['psr_corr'] - s['r_gt'] - clk_bias[sys]
        info = s['info_2b']
        n_b   = info['n_b']
        dist1 = info['dist1']
        dist2 = info['dist2']
        normal = np.array([info['ne'], info['nn'], info['nu']])

        # 1-bounce delta_L: 2 * d1 * cos(theta_i)
        n_norm = np.linalg.norm(normal)
        if n_norm > 0.1:
            n_unit = normal / n_norm
            d_sat  = sat_dir_enu(info['azim'], info['elev2b'])
            cos_theta = abs(float(np.dot(d_sat, n_unit)))
            dL_1b = 2.0 * dist1 * cos_theta
        else:
            dL_1b = float('nan')

        # 2-bounce delta_L: d1 + d2 (only valid when n_bounces == 2)
        dL_2b = dist1 + dist2 if n_b == 2 else float('nan')

        out_rows.append({
            'utc_t':          f'{s["utc_t"]:.3f}',
            'sat_id':         s['sat_id'],
            'sys':            s['sys'],
            'elevation_deg':  f'{s["elev"]:.2f}',
            'n_bounces':      n_b,
            'actual_error_m': f'{actual_error:.3f}',
            'dL_1b_m':        f'{dL_1b:.3f}' if not math.isnan(dL_1b) else '',
            'dL_2b_m':        f'{dL_2b:.3f}' if not math.isnan(dL_2b) else '',
            'dist1_m':        f'{dist1:.2f}',
            'dist2_m':        f'{dist2:.2f}',
        })

    if (ep_i + 1) % 100 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} epochs, {len(out_rows)} NLOS rows...', end='', flush=True)

print(f'\nDone.  NLOS rows: {len(out_rows)}  skip_gt: {epoch_skip}  skip_clk: {clk_fail}')
print(f'  NLOS key hits: {key_hits}  misses (treated LOS): {key_miss}')

if not out_rows:
    print('No output rows.  Check that unix_t in 2b CSV matches rinex_t from obs.')
    print(f'  Sample 2b keys: {list(nlos2b.keys())[:3]}')
    sys.exit(1)

# ── statistics ───────────────────────────────────────────────────────
actual = np.array([float(r['actual_error_m']) for r in out_rows])

rows_1b = [r for r in out_rows if r['dL_1b_m']]
rows_2b = [r for r in out_rows if r['dL_2b_m']]

a_1b = np.array([float(r['actual_error_m']) for r in rows_1b])
a_2b = np.array([float(r['actual_error_m']) for r in rows_2b])
d_1b = np.array([float(r['dL_1b_m'])        for r in rows_1b])
d_2b = np.array([float(r['dL_2b_m'])        for r in rows_2b])
e_1b = np.array([float(r['elevation_deg'])   for r in rows_1b])
e_2b = np.array([float(r['elevation_deg'])   for r in rows_2b])

r_1b = float(np.corrcoef(d_1b, a_1b)[0, 1]) if len(d_1b) > 2 else float('nan')
r_2b = float(np.corrcoef(d_2b, a_2b)[0, 1]) if len(d_2b) > 2 else float('nan')

print(f'\n=== Correlation comparison ===')
print(f'  1-bounce  n={len(rows_1b):4d}  r={r_1b:+.3f}'
      f'  actual mean={a_1b.mean():.2f}m  dL mean={d_1b.mean():.2f}m')
print(f'  2-bounce  n={len(rows_2b):4d}  r={r_2b:+.3f}'
      f'  actual mean={a_2b.mean():.2f}m  dL mean={d_2b.mean():.2f}m')

# elevation-stratified r values
elev_bins = [(10,20),(20,30),(30,45),(45,60),(60,90)]
print(f'\n  Elevation-stratified r (1b vs 2b):')
for lo, hi in elev_bins:
    m1 = (e_1b >= lo) & (e_1b < hi)
    m2 = (e_2b >= lo) & (e_2b < hi)
    r1 = float(np.corrcoef(d_1b[m1], a_1b[m1])[0,1]) if m1.sum() > 2 else float('nan')
    r2 = float(np.corrcoef(d_2b[m2], a_2b[m2])[0,1]) if m2.sum() > 2 else float('nan')
    print(f'    {lo:2d}-{hi:2d} deg:  1b r={r1:+.3f} (n={m1.sum()})  '
          f'2b r={r2:+.3f} (n={m2.sum()})')

# ── save CSV ─────────────────────────────────────────────────────────
COLS = ['utc_t','sat_id','sys','elevation_deg','n_bounces',
        'actual_error_m','dL_1b_m','dL_2b_m','dist1_m','dist2_m']
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS)
    w.writeheader()
    w.writerows(out_rows)
print(f'Saved CSV: {args.out_csv}')

# ── plots ────────────────────────────────────────────────────────────
def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

imgs = {}

# Fig 1: scatter actual vs dL (1b vs 2b side by side)
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
fig.suptitle('Actual pseudorange error vs predicted delta-L', fontsize=13)

clip = 200
for ax, xs, ys, es, r, label, color in [
    (axes[0], d_1b, a_1b, e_1b, r_1b, '1-bounce: 2*d1*cos(theta)', '#FF9800'),
    (axes[1], d_2b, a_2b, e_2b, r_2b, '2-bounce: d1+d2',           '#F44336'),
]:
    sc = ax.scatter(xs, ys, c=es, cmap='RdYlGn_r', s=6, alpha=0.4,
                    vmin=10, vmax=60)
    ax.set_xlim(-5, clip); ax.set_ylim(-clip, clip)
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.plot([0, clip], [0, clip], 'b--', lw=0.8, alpha=0.5, label='y=x')
    ax.set_xlabel(f'Predicted dL (m)')
    ax.set_ylabel('Actual error (m)')
    ax.set_title(f'{label}\nr = {r:+.3f}  n={len(xs)}')
    ax.legend(fontsize=8)
    plt.colorbar(sc, ax=ax, label='Elevation (deg)')
plt.tight_layout()
imgs['scatter'] = fig_to_b64(fig)

# Fig 2: elevation-stratified r comparison bar chart
fig, ax = plt.subplots(figsize=(9, 4.5))
bin_labels = [f'{lo}-{hi}' for lo,hi in elev_bins]
r1_list, r2_list, n1_list, n2_list = [], [], [], []
for lo, hi in elev_bins:
    m1 = (e_1b >= lo) & (e_1b < hi)
    m2 = (e_2b >= lo) & (e_2b < hi)
    r1_list.append(float(np.corrcoef(d_1b[m1],a_1b[m1])[0,1]) if m1.sum()>2 else 0)
    r2_list.append(float(np.corrcoef(d_2b[m2],a_2b[m2])[0,1]) if m2.sum()>2 else 0)
    n1_list.append(int(m1.sum()))
    n2_list.append(int(m2.sum()))

x = np.arange(len(bin_labels))
w = 0.35
bars1 = ax.bar(x - w/2, r1_list, w, label='1-bounce', color='#FF9800', alpha=0.85)
bars2 = ax.bar(x + w/2, r2_list, w, label='2-bounce', color='#F44336', alpha=0.85)
ax.axhline(0, color='k', lw=0.8)
ax.set_xticks(x)
ax.set_xticklabels([f'{l} deg\n(n1={n1_list[i]}, n2={n2_list[i]})'
                    for i, l in enumerate(bin_labels)])
ax.set_ylabel('Pearson r')
ax.set_title('Correlation r by elevation bin: 1-bounce vs 2-bounce')
ax.legend()
ax.set_ylim(-0.5, 0.8)
ax.grid(axis='y', alpha=0.3)
for bar in bars1:
    h = bar.get_height()
    ax.text(bar.get_x()+bar.get_width()/2, h+0.01, f'{h:+.2f}',
            ha='center', va='bottom', fontsize=8)
for bar in bars2:
    h = bar.get_height()
    ax.text(bar.get_x()+bar.get_width()/2, h+0.01, f'{h:+.2f}',
            ha='center', va='bottom', fontsize=8)
plt.tight_layout()
imgs['elev_r'] = fig_to_b64(fig)

# Fig 3: CDF of actual errors for 1b vs 2b groups
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
axes[0].plot(np.sort(a_1b), np.linspace(0,1,len(a_1b)), color='#FF9800',
             label=f'1-bounce (n={len(a_1b)}, mean={a_1b.mean():.1f}m)')
axes[0].plot(np.sort(a_2b), np.linspace(0,1,len(a_2b)), color='#F44336',
             label=f'2-bounce (n={len(a_2b)}, mean={a_2b.mean():.1f}m)')
axes[0].set_xlabel('Actual error (m)'); axes[0].set_ylabel('CDF')
axes[0].set_title('CDF of actual pseudorange errors')
axes[0].set_xlim(-100, 200); axes[0].legend(); axes[0].grid(alpha=0.3)

axes[1].plot(np.sort(d_1b), np.linspace(0,1,len(d_1b)), color='#FF9800',
             label=f'dL_1b (mean={d_1b.mean():.1f}m)')
axes[1].plot(np.sort(d_2b), np.linspace(0,1,len(d_2b)), color='#F44336',
             label=f'dL_2b (mean={d_2b.mean():.1f}m)')
axes[1].set_xlabel('Predicted dL (m)'); axes[1].set_ylabel('CDF')
axes[1].set_title('CDF of predicted delta-L values')
axes[1].set_xlim(0, 80); axes[1].legend(); axes[1].grid(alpha=0.3)
plt.tight_layout()
imgs['cdf'] = fig_to_b64(fig)

# Fig 4: actual error distribution by elevation for 2b group
fig, ax = plt.subplots(figsize=(8, 4.5))
for lo, hi in elev_bins:
    m = (e_2b >= lo) & (e_2b < hi)
    if m.sum() > 3:
        ax.hist(a_2b[m], bins=30, alpha=0.5, label=f'{lo}-{hi} deg (n={m.sum()})',
                density=True, range=(-100, 200))
ax.set_xlabel('Actual error (m)'); ax.set_ylabel('Density')
ax.set_title('Actual error distribution by elevation (2-bounce group)')
ax.legend(fontsize=8); ax.grid(alpha=0.3)
plt.tight_layout()
imgs['elev_dist'] = fig_to_b64(fig)

# ── generate HTML ────────────────────────────────────────────────────
html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>1-bounce vs 2-bounce Correlation Analysis</title>
<style>
body{{font-family:sans-serif;margin:20px;background:#f5f5f5}}
h1{{color:#333}} h2{{color:#555;margin-top:28px}}
img{{max-width:100%;border:1px solid #ddd;border-radius:6px;margin:8px 0}}
table{{border-collapse:collapse;margin:10px 0}}
td,th{{border:1px solid #ccc;padding:6px 14px;text-align:center}}
th{{background:#4a90d9;color:white}}
tr:nth-child(even){{background:#eef}}
.key{{background:#fff3cd;padding:14px;border-left:4px solid #ffc107;margin:12px 0;font-size:15px}}
.better{{color:green;font-weight:bold}}
.worse{{color:red}}
</style></head><body>
<h1>1-Bounce vs 2-Bounce delta-L: Correlation with Actual Errors</h1>
<div class="key">
<b>Overall correlation:</b><br>
&nbsp;&nbsp; 1-bounce model (dL = 2*d1*cos theta): &nbsp;
  <span class="{'better' if r_1b > r_2b else 'worse'}">r = {r_1b:+.3f}</span>
  &nbsp; n = {len(rows_1b)}<br>
&nbsp;&nbsp; 2-bounce model (dL = d1 + d2): &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <span class="{'better' if r_2b > r_1b else 'worse'}">r = {r_2b:+.3f}</span>
  &nbsp; n = {len(rows_2b)}<br><br>
<b>{'2-bounce is a stronger predictor' if r_2b > r_1b else '1-bounce is a stronger predictor'}</b>
 (higher r = better linear prediction of actual NLOS error)
</div>

<table>
<tr><th>Model</th><th>n</th><th>r (Pearson)</th>
    <th>actual mean (m)</th><th>actual std (m)</th>
    <th>predicted dL mean (m)</th></tr>
<tr><td>1-bounce: 2*d1*cos(theta)</td><td>{len(rows_1b)}</td>
    <td>{r_1b:+.3f}</td>
    <td>{a_1b.mean():.2f}</td><td>{a_1b.std():.2f}</td>
    <td>{d_1b.mean():.2f}</td></tr>
<tr><td>2-bounce: d1+d2</td><td>{len(rows_2b)}</td>
    <td>{r_2b:+.3f}</td>
    <td>{a_2b.mean():.2f}</td><td>{a_2b.std():.2f}</td>
    <td>{d_2b.mean():.2f}</td></tr>
</table>

<h2>1. Scatter: Actual Error vs Predicted delta-L</h2>
<img src="data:image/png;base64,{imgs['scatter']}">

<h2>2. Correlation r by Elevation Bin</h2>
<img src="data:image/png;base64,{imgs['elev_r']}">

<h2>3. CDF: Actual Errors and Predicted delta-L</h2>
<img src="data:image/png;base64,{imgs['cdf']}">

<h2>4. Actual Error Distribution by Elevation (2-bounce group)</h2>
<img src="data:image/png;base64,{imgs['elev_dist']}">

</body></html>"""

with open(args.out_html, 'w') as f:
    f.write(html)
print(f'Saved HTML: {args.out_html}')
