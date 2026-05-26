#!/usr/bin/env python3
"""
lidar_step7b_2bounce_spp.py
Multi-GNSS WLS SPP with elevation-aware 2-bounce pseudorange correction.

Uses lidar_nlos_prediction_2b.csv (Step4 output) as the sole NLOS source.
Compares four modes:
  baseline         — all satellites, no correction
  exclusion        — drop all NLOS satellites
  1bounce_corr     — subtract dL_1b = 2*d1*cos(theta), low weight
  2bounce_corr     — elevation-aware: apply dL_2b = d1+d2 selectively
                     based on per-bin correlation from Step 8b analysis:
                       30-45 deg 2b: r=+0.705 → weight*0.40
                       10-20 deg 2b: r=+0.285 → weight*0.20
                       45-90 deg 2b: r=+0.184 → weight*0.15
                       20-30 deg 2b: r=-0.216 → EXCLUDE (unreliable)
                       1-bounce only: weight*0.05

Usage (inside container):
  python3 lidar_step7b_2bounce_spp.py \\
    --obs      /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
    --nav_gps  /root/urbannav_gnss/hksc137c.21n \\
    --nav_bds  /root/urbannav_gnss/hksc137c.21f \\
    --nav_gal  /root/urbannav_gnss/hksc137c.21l \\
    --nlos_2b  /root/lidar_nlos_prediction_2b.csv \\
    --gt       /root/urbannav_gt.txt \\
    --out_csv  /root/spp_2bounce_results.csv \\
    --out_html /root/spp_2bounce_report.html
"""

import argparse, base64, csv, io, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

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
ap.add_argument('--obs',      default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps',  default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds',  default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal',  default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--nlos_2b',  default='/root/lidar_nlos_prediction_2b.csv')
ap.add_argument('--gt',       default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',  default='/root/spp_2bounce_results.csv')
ap.add_argument('--out_html', default='/root/spp_2bounce_report.html')
ap.add_argument('--min_elev', type=float, default=10.0)
ap.add_argument('--gt_tol',   type=float, default=10.0)
args = ap.parse_args()

# ── helpers ──────────────────────────────────────────────────────────
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

def sat_dir_enu(azim_deg, elev_deg):
    az, el = math.radians(azim_deg), math.radians(elev_deg)
    return np.array([math.sin(az)*math.cos(el),
                     math.cos(az)*math.cos(el),
                     math.sin(el)])

# ── WLS SPP solver (multi-constellation) ────────────────────────────
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

# ── load 2b NLOS data ────────────────────────────────────────────────
print(f'Loading 2b data: {args.nlos_2b}')
nlos2b = {}
with open(args.nlos_2b) as f:
    for row in csv.DictReader(f):
        ne = float(row['normal_e']); nn = float(row['normal_n']); nu = float(row['normal_u'])
        nlos2b[(row['unix_t'], row['sat_id'])] = {
            'is_nlos': int(row['lidar_nlos']),
            'n_b':     int(row['n_bounces']),
            'dist1':   float(row['hit_dist_m']),
            'dist2':   float(row['hit_dist2_m']),
            'ne': ne, 'nn': nn, 'nu': nu,
            'azim':    float(row['azimuth_deg']),
            'elev2b':  float(row['elevation_deg']),
        }
print(f'  {len(nlos2b)} records  '
      f'NLOS={sum(1 for v in nlos2b.values() if v["is_nlos"])}  '
      f'2-bounce={sum(1 for v in nlos2b.values() if v["n_b"]==2)}')

# ── load GT ──────────────────────────────────────────────────────────
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
            gt_times.append(t); gt_lats.append(dms_to_deg(p[3],p[4],p[5]))
            gt_lons.append(dms_to_deg(p[6],p[7],p[8])); gt_alts.append(float(p[9]))
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
    idx = min(max(idx,0), len(gt_times)-1)
    if idx > 0 and abs(gt_times[idx-1]-utc_t) < abs(gt_times[idx]-utc_t):
        idx -= 1
    return (gt_lats[idx],gt_lons[idx],gt_alts[idx]) if abs(gt_times[idx]-utc_t)<=args.gt_tol else None

# ── load nav + obs ───────────────────────────────────────────────────
print('Loading navigation messages...')
ephem = {}
ephem.update(load_nav(args.nav_gps,'G'))
ephem.update(load_nav(args.nav_bds,'C'))
ephem.update(load_nav(args.nav_gal,'E'))
print(f'  {sum(len(v) for v in ephem.values())} ephemerides')

print(f'Loading obs: {args.obs}')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} epochs')

x0_ecef = np.array(llh_to_ecef(22.3198, 114.2095, 20.0))

# ── correction weight lookup by elevation bin (from Step 8b r analysis) ──
# 2-bounce correlation results:
#   10-20 deg: r=+0.285  20-30 deg: r=-0.216  30-45 deg: r=+0.705
#   45-60 deg: r=+0.184  60-90 deg: r=-0.111
def get_2b_weight_factor(elev):
    if   30 <= elev < 45: return 0.40   # r=+0.705, highest confidence
    elif 10 <= elev < 20: return 0.20   # r=+0.285
    elif 45 <= elev < 60: return 0.15   # r=+0.184
    elif 60 <= elev:      return 0.10   # r=-0.111, weak
    else:                 return 0.0    # 20-30 deg: r=-0.216 → exclude

MODES = ('baseline', 'exclusion', '1bounce_corr', '2bounce_corr')

# ── main processing loop ─────────────────────────────────────────────
print(f'\nRunning SPP (4 modes, min_elev={args.min_elev} deg)...')
results = []
no_gt   = 0

for ep_i, epoch in enumerate(obs_epochs):
    rinex_t = epoch.time_unix
    utc_t   = rinex_t - LEAP_SECONDS

    gt_match = match_gt(utc_t)
    if gt_match is None:
        no_gt += 1; continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    sat_data = []
    for obs in epoch.obs_list:
        sat_id   = obs.sat_id
        sys_char = obs.sys if obs.sys else sat_id[0]
        if sys_char not in ('G','C','E'):
            continue
        eph = find_closest_ephem(ephem.get(sat_id,[]), rinex_t)
        if eph is None:
            continue
        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_t)
        if sat_ecef_raw is None:
            continue

        psr_raw = 0.0
        for k in PSR_KEYS.get(sys_char,['C1C']):
            if k in obs.pseudorange and obs.pseudorange[k] > 0:
                psr_raw = obs.pseudorange[k]; break
        if psr_raw <= 0:
            continue

        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), psr_raw/C_LIGHT)
        diff_rx = sat_ecef_s - x0_ecef
        r_approx = np.linalg.norm(diff_rx)
        lat_r, lon_r = math.radians(gt_lat), math.radians(gt_lon)
        up = np.array([math.cos(lat_r)*math.cos(lon_r),
                       math.cos(lat_r)*math.sin(lon_r),
                       math.sin(lat_r)])
        elev = math.degrees(math.asin(np.clip(np.dot(diff_rx/r_approx, up),-1,1)))
        if elev < args.min_elev:
            continue

        psr_corr = psr_raw + C_LIGHT*dt_sv - tropo_delay(elev)
        w_base   = math.sin(math.radians(elev))**2

        # look up 2b info
        key_r = (f'{rinex_t:.3f}', sat_id)
        key_u = (f'{utc_t:.3f}',   sat_id)
        info  = nlos2b.get(key_r) or nlos2b.get(key_u)

        if info and info['is_nlos']:
            n_b   = info['n_b']
            dist1 = info['dist1']
            dist2 = info['dist2']
            normal = np.array([info['ne'], info['nn'], info['nu']])
            n_norm = np.linalg.norm(normal)

            # 1-bounce dL
            if n_norm > 0.1:
                n_unit = normal / n_norm
                d_sat  = sat_dir_enu(info['azim'], info['elev2b'])
                dL_1b  = 2.0 * dist1 * abs(float(np.dot(d_sat, n_unit)))
            else:
                dL_1b = 0.0

            # 2-bounce dL
            dL_2b = dist1 + dist2 if n_b == 2 else 0.0
        else:
            n_b = 0; dL_1b = 0.0; dL_2b = 0.0

        sat_data.append({
            'sat_id':  sat_id, 'sys': sys_char,
            'sat_ecef': sat_ecef_s,
            'psr_corr': psr_corr,
            'elev':     elev,
            'w_base':   w_base,
            'is_nlos':  1 if (info and info['is_nlos']) else 0,
            'n_b':      n_b,
            'dL_1b':    dL_1b,
            'dL_2b':    dL_2b,
        })

    if len(sat_data) < 4:
        continue

    mode_results = {}
    for mode in MODES:
        sats_info = []
        for s in sat_data:
            is_nlos = s['is_nlos']
            elev    = s['elev']
            n_b     = s['n_b']

            if mode == 'baseline':
                w   = s['w_base']
                psr = s['psr_corr']

            elif mode == 'exclusion':
                w   = 0.0 if is_nlos else s['w_base']
                psr = s['psr_corr']

            elif mode == '1bounce_corr':
                if is_nlos:
                    psr = s['psr_corr'] - s['dL_1b']
                    w   = s['w_base'] * 0.05
                else:
                    psr = s['psr_corr']
                    w   = s['w_base']

            else:  # 2bounce_corr — elevation-aware
                if is_nlos:
                    if n_b == 2:
                        wf = get_2b_weight_factor(elev)
                        if wf > 0:
                            psr = s['psr_corr'] - s['dL_2b']
                            w   = s['w_base'] * wf
                        else:
                            # 20-30 deg: r negative → exclude
                            psr = s['psr_corr']
                            w   = 0.0
                    else:
                        # n_b==1: apply 1b correction, very low weight
                        psr = s['psr_corr'] - s['dL_1b']
                        w   = s['w_base'] * 0.03
                else:
                    psr = s['psr_corr']
                    w   = s['w_base']

            sats_info.append({'sat_ecef': s['sat_ecef'], 'psr_corr': psr,
                               'weight': w, 'elevation': elev, 'sys': s['sys']})

        pos, _, pdop = wls_spp_multi(sats_info, x0_ecef)
        if pos is None:
            mode_results[mode] = None; continue

        enu  = ecef_to_enu(pos, gt_ecef, gt_lat, gt_lon)
        err_h  = math.sqrt(enu[0]**2 + enu[1]**2)
        err_v  = abs(enu[2])
        err_3d = math.sqrt(err_h**2 + err_v**2)
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
        'n_total': len(sat_data),
        'n_nlos':  sum(1 for s in sat_data if s['is_nlos']),
        'n_2b':    sum(1 for s in sat_data if s['n_b']==2),
        **{f'{m}_{k}': (mode_results[m][k] if mode_results[m] else None)
           for m in MODES for k in ('err_h','err_v','err_3d','pdop','n_used')},
    })

    if (ep_i+1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} epochs, {len(results)} valid...', end='', flush=True)

print(f'\nDone: {len(results)} valid epochs  (no_gt={no_gt})')

# ── statistics ────────────────────────────────────────────────────────
def stats(vals):
    v = [x for x in vals if x is not None]
    if not v: return dict(n=0, mean=float('nan'), rms=float('nan'), p50=float('nan'), p95=float('nan'))
    a = np.array(v)
    return dict(n=len(v), mean=float(np.mean(a)), rms=float(np.sqrt(np.mean(a**2))),
                p50=float(np.percentile(a,50)), p95=float(np.percentile(a,95)))

st = {m: stats([r[f'{m}_err_h'] for r in results]) for m in MODES}

print('\n=== Horizontal Error Summary ===')
print(f'  {"Mode":<18}  {"n":>4}  {"mean":>7}  {"RMS":>7}  {"50th":>7}  {"95th":>7}')
for m in MODES:
    s = st[m]
    diff = f'  ({s["mean"]-st["baseline"]["mean"]:+.1f}m vs baseline)' if m != 'baseline' else ''
    print(f'  {m:<18}  {s["n"]:4d}  {s["mean"]:7.2f}m  {s["rms"]:7.2f}m  '
          f'{s["p50"]:7.2f}m  {s["p95"]:7.2f}m{diff}')

# ── save CSV ─────────────────────────────────────────────────────────
COLS = ['utc_t','n_total','n_nlos','n_2b'] + \
       [f'{m}_{k}' for m in MODES for k in ('err_h','err_v','err_3d','pdop','n_used')]
with open(args.out_csv,'w',newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS, extrasaction='ignore')
    w.writeheader(); w.writerows(results)
print(f'Saved: {args.out_csv}')

# ── plots ─────────────────────────────────────────────────────────────
def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

imgs = {}
colors = {'baseline':'#F44336','exclusion':'#FF9800',
          '1bounce_corr':'#9C27B0','2bounce_corr':'#2196F3'}
labels = {'baseline':'Baseline','exclusion':'Exclusion',
          '1bounce_corr':'1-bounce corr','2bounce_corr':'2-bounce corr'}

# Fig 1: CDF comparison
fig, ax = plt.subplots(figsize=(9,5))
for m in MODES:
    vals = sorted(r[f'{m}_err_h'] for r in results if r[f'{m}_err_h'] is not None)
    if vals:
        lw = 2.5 if m == '2bounce_corr' else 1.5
        ax.plot(vals, np.linspace(0,1,len(vals)),
                color=colors[m], lw=lw, label=f'{labels[m]} (mean={st[m]["mean"]:.1f}m)')
ax.set_xlabel('Horizontal error (m)'); ax.set_ylabel('CDF')
ax.set_title('SPP Horizontal Error CDF — 4 Mode Comparison')
ax.set_xlim(0, 200); ax.legend(); ax.grid(True, alpha=0.3)
imgs['cdf'] = fig_to_b64(fig)

# Fig 2: bar chart of statistics
fig, ax = plt.subplots(figsize=(10,5))
metrics  = ['mean','rms','p50','p95']
mlabels  = ['Mean','RMS','50th pct','95th pct']
x = np.arange(len(metrics)); w = 0.2
for i, m in enumerate(MODES):
    vals = [st[m][k] for k in metrics]
    bars = ax.bar(x + i*w, vals, w, label=labels[m], color=colors[m], alpha=0.85)
ax.set_xticks(x + 1.5*w); ax.set_xticklabels(mlabels)
ax.set_ylabel('Error (m)'); ax.set_title('SPP Error Statistics by Mode')
ax.legend(); ax.grid(axis='y', alpha=0.3)
imgs['bar'] = fig_to_b64(fig)

# Fig 3: time series
fig, ax = plt.subplots(figsize=(13,4.5))
for m in MODES:
    ts = [(r['utc_t'], r[f'{m}_err_h']) for r in results if r[f'{m}_err_h'] is not None]
    if ts:
        lw = 2.0 if m == '2bounce_corr' else 1.0
        ax.plot([t for t,_ in ts], [e for _,e in ts],
                color=colors[m], lw=lw, alpha=0.8, label=labels[m])
ax.set_xlabel('UTC time (s)'); ax.set_ylabel('Horizontal error (m)')
ax.set_title('SPP Horizontal Error — Time Series')
ax.legend(); ax.grid(True, alpha=0.3); ax.set_ylim(0, 300)
imgs['ts'] = fig_to_b64(fig)

# Fig 4: improvement per epoch (2bounce_corr vs baseline)
fig, ax = plt.subplots(figsize=(13,4))
paired = [(r['utc_t'], r['baseline_err_h'] - r['2bounce_corr_err_h'])
          for r in results
          if r['baseline_err_h'] is not None and r['2bounce_corr_err_h'] is not None]
if paired:
    ts_p = [t for t,_ in paired]; impr = [d for _,d in paired]
    pos_frac = sum(1 for d in impr if d > 0) / len(impr) * 100
    ax.bar(ts_p, impr, width=2, color=['#2196F3' if d>0 else '#F44336' for d in impr], alpha=0.7)
    ax.axhline(0, color='k', lw=0.8)
    ax.set_xlabel('UTC time (s)'); ax.set_ylabel('Improvement (m)\n(positive = 2bounce better)')
    ax.set_title(f'Per-epoch improvement: 2-bounce corr vs baseline  '
                 f'(better in {pos_frac:.0f}% of epochs)')
    ax.grid(axis='y', alpha=0.3)
imgs['improvement'] = fig_to_b64(fig)

# ── HTML ─────────────────────────────────────────────────────────────
def delta_str(m):
    d = st[m]['mean'] - st['baseline']['mean']
    sign = '&darr;' if d < 0 else '&uarr;'
    cls  = 'better' if d < 0 else 'worse'
    return f'<span class="{cls}">{sign}{abs(d):.1f}m</span>'

html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Step 7b — 2-Bounce SPP Correction</title>
<style>
body{{font-family:sans-serif;margin:20px;background:#f4f6fb;color:#333}}
h1{{color:#1a237e;border-bottom:3px solid #3f51b5;padding-bottom:8px}}
h2{{color:#283593;margin-top:26px}}
img{{max-width:100%;border:1px solid #ddd;border-radius:6px;margin:8px 0}}
table{{border-collapse:collapse;width:100%}}
th{{background:#3f51b5;color:white;padding:8px 14px;text-align:left}}
td{{padding:7px 14px;border-bottom:1px solid #e8e8e8}}
tr:nth-child(even){{background:#f5f7ff}}
.better{{color:#2e7d32;font-weight:bold}} .worse{{color:#c62828;font-weight:bold}}
.key{{background:#e8f5e9;padding:14px;border-left:4px solid #43a047;margin:12px 0}}
.formula{{background:#f3f4f8;border-left:4px solid #3f51b5;
          padding:10px 16px;font-family:monospace;margin:10px 0;font-size:.92em}}
</style></head><body>
<h1>Step 7b — 2-Bounce Pseudorange Correction in WLS SPP</h1>
<p>UrbanNav HK Medium-Urban-1 &nbsp;|&nbsp; Valid epochs: {len(results)} &nbsp;|&nbsp;
Elevation cutoff: {args.min_elev} deg</p>

<div class="key">
<b>Key result:</b><br>
Baseline mean horizontal error: <b>{st['baseline']['mean']:.1f} m</b><br>
2-bounce correction: <b>{st['2bounce_corr']['mean']:.1f} m</b>
&nbsp; {delta_str('2bounce_corr')} vs baseline<br>
Exclusion: {st['exclusion']['mean']:.1f} m &nbsp; {delta_str('exclusion')}<br>
1-bounce correction: {st['1bounce_corr']['mean']:.1f} m &nbsp; {delta_str('1bounce_corr')}
</div>

<div class="formula">
<b>2-bounce correction strategy</b> (based on Step 8b per-bin r analysis):<br>
&nbsp; 30-45 deg, n_bounces=2: psr -= (d1+d2), weight*0.40 &nbsp; [r=+0.705]<br>
&nbsp; 10-20 deg, n_bounces=2: psr -= (d1+d2), weight*0.20 &nbsp; [r=+0.285]<br>
&nbsp; 45-60 deg, n_bounces=2: psr -= (d1+d2), weight*0.15 &nbsp; [r=+0.184]<br>
&nbsp; 20-30 deg, n_bounces=2: EXCLUDE &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [r=-0.216, unreliable]<br>
&nbsp; n_bounces=1 only: psr -= dL_1b, weight*0.03
</div>

<table>
<tr><th>Mode</th><th>n</th><th>Mean (m)</th><th>RMS (m)</th>
    <th>50th (m)</th><th>95th (m)</th><th>vs Baseline</th></tr>
{''.join(
  f'<tr><td><b>{labels[m]}</b></td><td>{st[m]["n"]}</td>'
  f'<td>{st[m]["mean"]:.2f}</td><td>{st[m]["rms"]:.2f}</td>'
  f'<td>{st[m]["p50"]:.2f}</td><td>{st[m]["p95"]:.2f}</td>'
  f'<td>{"&mdash;" if m=="baseline" else delta_str(m)}</td></tr>'
  for m in MODES
)}
</table>

<h2>1. CDF of Horizontal Error</h2>
<img src="data:image/png;base64,{imgs['cdf']}">

<h2>2. Error Statistics by Mode</h2>
<img src="data:image/png;base64,{imgs['bar']}">

<h2>3. Time Series</h2>
<img src="data:image/png;base64,{imgs['ts']}">

<h2>4. Per-Epoch Improvement (2-bounce vs Baseline)</h2>
<img src="data:image/png;base64,{imgs['improvement']}">

</body></html>"""

with open(args.out_html,'w') as f:
    f.write(html)
print(f'Saved: {args.out_html}')
