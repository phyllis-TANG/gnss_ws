#!/usr/bin/env python3
"""
lidar_step8a_cn0_nlos.py
C/N0-based NLOS detection for GPS L1 (S1C observable).

Method:
  1. For every GPS satellite observation, read S1C (C/N0 in dB-Hz).
  2. Build an elevation-dependent expected C/N0 model using binned medians
     across ALL observations (robust, data-driven, no functional-form assumption).
  3. cn0_deficit = expected_cn0(elev) - actual_cn0  [dB-Hz]
     Large positive deficit  =>  signal attenuated relative to sky  =>  NLOS candidate.
  4. Flag deficit > cn0_thresh as CN0-NLOS.
  5. Cross-compare with DD labels (dd_nlos_labels.csv) and LiDAR 2-bounce labels.
  6. Output CSV + HTML report.

Output CSV columns:
  rinex_t, sat_id, elev_deg, cn0, cn0_expected, cn0_deficit,
  nlos_cn0, nlos_dd, nlos_lidar, nlos_any2 (any 2 of 3 agree NLOS)

Usage (inside container):
  python3 lidar_step8a_cn0_nlos.py \\
    --obs       /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
    --nav_gps   /root/urbannav_gnss/hksc137c.21n \\
    --dd_labels /root/dd_nlos_labels.csv \\
    --nlos_2b   /root/lidar_nlos_prediction_2b.csv \\
    --gt        /root/urbannav_gt.txt \\
    --out_csv   /root/cn0_nlos_labels.csv \\
    --out_html  /root/cn0_nlos_report.html
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
ELEV_BIN_DEG = 5
ELEV_BINS    = np.arange(0, 95, ELEV_BIN_DEG)

ap = argparse.ArgumentParser()
ap.add_argument('--obs',       default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps',   default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--dd_labels', default='/root/dd_nlos_labels.csv')
ap.add_argument('--nlos_2b',   default='/root/lidar_nlos_prediction_2b.csv')
ap.add_argument('--gt',        default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',   default='/root/cn0_nlos_labels.csv')
ap.add_argument('--out_html',  default='/root/cn0_nlos_report.html')
ap.add_argument('--cn0_thresh',type=float, default=6.0)
ap.add_argument('--min_elev',  type=float, default=10.0)
ap.add_argument('--gt_tol',    type=float, default=10.0)
args = ap.parse_args()

def sagnac_correct(sat_ecef, travel_time_s):
    theta = OMEGA_E * travel_time_s
    c, s  = math.cos(theta), math.sin(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ sat_ecef

def dms_to_deg(d, m, s):
    return float(d) + float(m)/60 + float(s)/3600

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

print('Loading GPS nav...')
try:
    raw_nav = read_rinex_nav(args.nav_gps)
except Exception as e:
    print(f'  [ERR] {e}'); sys.exit(1)
ephem = {}
for k, v in raw_nav.items():
    nk = f'G{k:02d}' if isinstance(k, int) else k
    ephem[nk] = v
print(f'  {sum(len(v) for v in ephem.values())} GPS ephemerides')

print(f'Loading DD labels: {args.dd_labels}')
dd_map = {}
with open(args.dd_labels) as f:
    for row in csv.DictReader(f):
        if row['sys'] != 'G': continue
        dd_map[(round(float(row['rinex_t']), 3), row['sat_id'])] = int(row['nlos_dd'])
print(f'  {len(dd_map)} GPS DD records')

print(f'Loading LiDAR 2b labels: {args.nlos_2b}')
lidar_map = {}
try:
    with open(args.nlos_2b) as f:
        for row in csv.DictReader(f):
            t_key = round(float(row.get('rinex_t', row.get('timestamp', 0))), 3)
            sid   = row.get('sat_id', row.get('prn', ''))
            lidar_map[(t_key, sid)] = 1
    print(f'  {len(lidar_map)} LiDAR NLOS records')
except FileNotFoundError:
    print('  [WARN] LiDAR 2b file not found')

print(f'Loading obs: {args.obs}')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} epochs')

x0_ecef = np.array(llh_to_ecef(22.3198, 114.2095, 20.0))

print('\nPass 1: collecting C/N0 vs elevation data...')
all_elev = []
all_cn0  = []

for epoch in obs_epochs:
    rinex_t = epoch.time_unix
    utc_t   = rinex_t - LEAP_SECONDS
    gt_match = match_gt(utc_t)
    if gt_match is None: continue
    gt_lat, gt_lon, gt_alt = gt_match

    for obs in epoch.obs_list:
        if obs.sys != 'G': continue
        cn0_val = obs.cn0.get('S1C') if obs.cn0 else None
        if cn0_val is None or cn0_val <= 0: continue
        eph = find_closest_ephem(ephem.get(obs.sat_id, []), rinex_t)
        if eph is None: continue
        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_t)
        if sat_ecef_raw is None: continue
        psr_raw = obs.pseudorange.get('C1C', 0.0)
        if psr_raw <= 0: continue
        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), psr_raw/C_LIGHT)
        diff = sat_ecef_s - x0_ecef
        lat_r, lon_r = math.radians(gt_lat), math.radians(gt_lon)
        up = np.array([math.cos(lat_r)*math.cos(lon_r),
                       math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)])
        elev = math.degrees(math.asin(np.clip(np.dot(diff/np.linalg.norm(diff), up), -1, 1)))
        if elev < args.min_elev: continue
        all_elev.append(elev)
        all_cn0.append(cn0_val)

all_elev = np.array(all_elev)
all_cn0  = np.array(all_cn0)
print(f'  {len(all_cn0)} GPS L1 C/N0 observations')
print(f'  C/N0 range: {all_cn0.min():.1f} - {all_cn0.max():.1f} dB-Hz  median={np.median(all_cn0):.1f}')

print('\nBuilding C/N0 elevation model (binned medians)...')
bin_centers = []
bin_medians = []
bin_p25     = []
bin_p75     = []

for b in ELEV_BINS:
    mask = (all_elev >= b) & (all_elev < b + ELEV_BIN_DEG)
    if mask.sum() < 3: continue
    vals = all_cn0[mask]
    bin_centers.append(b + ELEV_BIN_DEG/2)
    bin_medians.append(float(np.median(vals)))
    bin_p25.append(float(np.percentile(vals, 25)))
    bin_p75.append(float(np.percentile(vals, 75)))
    print(f'  elev {b:2d}-{b+ELEV_BIN_DEG:2d}deg  n={mask.sum():4d}  '
          f'median={np.median(vals):.1f}  IQR=[{np.percentile(vals,25):.1f}, {np.percentile(vals,75):.1f}]')

bin_centers = np.array(bin_centers)
bin_medians = np.array(bin_medians)

def expected_cn0(elev_deg):
    if len(bin_centers) == 0: return float('nan')
    return float(np.interp(elev_deg, bin_centers, bin_medians,
                            left=bin_medians[0], right=bin_medians[-1]))

print(f'\nPass 2: computing C/N0 deficits (threshold={args.cn0_thresh} dB-Hz)...')
out_rows = []

for epoch in obs_epochs:
    rinex_t     = epoch.time_unix
    utc_t       = rinex_t - LEAP_SECONDS
    rinex_t_key = round(rinex_t, 3)
    gt_match = match_gt(utc_t)
    if gt_match is None: continue
    gt_lat, gt_lon, gt_alt = gt_match

    for obs in epoch.obs_list:
        if obs.sys != 'G': continue
        cn0_val = obs.cn0.get('S1C') if obs.cn0 else None
        if cn0_val is None or cn0_val <= 0: continue
        eph = find_closest_ephem(ephem.get(obs.sat_id, []), rinex_t)
        if eph is None: continue
        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_t)
        if sat_ecef_raw is None: continue
        psr_raw = obs.pseudorange.get('C1C', 0.0)
        if psr_raw <= 0: continue
        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), psr_raw/C_LIGHT)
        diff = sat_ecef_s - x0_ecef
        lat_r, lon_r = math.radians(gt_lat), math.radians(gt_lon)
        up = np.array([math.cos(lat_r)*math.cos(lon_r),
                       math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)])
        elev = math.degrees(math.asin(np.clip(np.dot(diff/np.linalg.norm(diff), up), -1, 1)))
        if elev < args.min_elev: continue

        cn0_exp    = expected_cn0(elev)
        deficit    = cn0_exp - cn0_val
        nlos_cn0   = 1 if deficit > args.cn0_thresh else 0
        nlos_dd    = dd_map.get((rinex_t_key, obs.sat_id), 0)
        nlos_lidar = lidar_map.get((rinex_t_key, obs.sat_id), 0)
        nlos_any2  = 1 if (nlos_cn0 + nlos_dd + nlos_lidar) >= 2 else 0

        out_rows.append({
            'rinex_t':      f'{rinex_t:.3f}',
            'utc_t':        f'{utc_t:.3f}',
            'sat_id':       obs.sat_id,
            'elev_deg':     f'{elev:.2f}',
            'cn0':          f'{cn0_val:.1f}',
            'cn0_expected': f'{cn0_exp:.2f}',
            'cn0_deficit':  f'{deficit:.2f}',
            'nlos_cn0':     nlos_cn0,
            'nlos_dd':      nlos_dd,
            'nlos_lidar':   nlos_lidar,
            'nlos_any2':    nlos_any2,
        })

print(f'  {len(out_rows)} GPS L1 observations processed')

n_total = len(out_rows)
n_cn0   = sum(1 for r in out_rows if r['nlos_cn0'])
n_dd    = sum(1 for r in out_rows if r['nlos_dd'])
n_lidar = sum(1 for r in out_rows if r['nlos_lidar'])
n_any2  = sum(1 for r in out_rows if r['nlos_any2'])

print(f'\n=== NLOS label summary ===')
print(f'  Total GPS obs:     {n_total}')
print(f'  CN0-flagged NLOS:  {n_cn0}  ({100*n_cn0/n_total:.1f}%)')
print(f'  DD-flagged NLOS:   {n_dd}   ({100*n_dd/n_total:.1f}%)')
print(f'  LiDAR-flagged:     {n_lidar} ({100*n_lidar/n_total:.1f}%)')
print(f'  Any-2 consensus:   {n_any2}  ({100*n_any2/n_total:.1f}%)')

tp_cl = sum(1 for r in out_rows if r['nlos_cn0'] and r['nlos_lidar'])
fp_cl = sum(1 for r in out_rows if r['nlos_cn0'] and not r['nlos_lidar'])
fn_cl = sum(1 for r in out_rows if not r['nlos_cn0'] and r['nlos_lidar'])
tn_cl = sum(1 for r in out_rows if not r['nlos_cn0'] and not r['nlos_lidar'])
prec_cl = tp_cl/(tp_cl+fp_cl) if (tp_cl+fp_cl) else 0
rec_cl  = tp_cl/(tp_cl+fn_cl) if (tp_cl+fn_cl) else 0
f1_cl   = 2*prec_cl*rec_cl/(prec_cl+rec_cl) if (prec_cl+rec_cl) else 0

tp_dl = sum(1 for r in out_rows if r['nlos_dd'] and r['nlos_lidar'])
fp_dl = sum(1 for r in out_rows if r['nlos_dd'] and not r['nlos_lidar'])
fn_dl = sum(1 for r in out_rows if not r['nlos_dd'] and r['nlos_lidar'])
tn_dl = sum(1 for r in out_rows if not r['nlos_dd'] and not r['nlos_lidar'])
prec_dl = tp_dl/(tp_dl+fp_dl) if (tp_dl+fp_dl) else 0
rec_dl  = tp_dl/(tp_dl+fn_dl) if (tp_dl+fn_dl) else 0
f1_dl   = 2*prec_dl*rec_dl/(prec_dl+rec_dl) if (prec_dl+rec_dl) else 0

tp_a2 = sum(1 for r in out_rows if r['nlos_any2'] and r['nlos_lidar'])
fp_a2 = sum(1 for r in out_rows if r['nlos_any2'] and not r['nlos_lidar'])
fn_a2 = sum(1 for r in out_rows if not r['nlos_any2'] and r['nlos_lidar'])
tn_a2 = sum(1 for r in out_rows if not r['nlos_any2'] and not r['nlos_lidar'])
prec_a2 = tp_a2/(tp_a2+fp_a2) if (tp_a2+fp_a2) else 0
rec_a2  = tp_a2/(tp_a2+fn_a2) if (tp_a2+fn_a2) else 0
f1_a2   = 2*prec_a2*rec_a2/(prec_a2+rec_a2) if (prec_a2+rec_a2) else 0

print(f'\n=== Confusion matrices (vs LiDAR ground truth) ===')
print(f'  CN0  (thresh={args.cn0_thresh}dB): '
      f'TP={tp_cl} FP={fp_cl} FN={fn_cl} TN={tn_cl}  '
      f'Prec={prec_cl:.3f} Rec={rec_cl:.3f} F1={f1_cl:.3f}')
print(f'  DD   (thresh=5m):            '
      f'TP={tp_dl} FP={fp_dl} FN={fn_dl} TN={tn_dl}  '
      f'Prec={prec_dl:.3f} Rec={rec_dl:.3f} F1={f1_dl:.3f}')
print(f'  Any2 (CN0 OR DD >=2/3):      '
      f'TP={tp_a2} FP={fp_a2} FN={fn_a2} TN={tn_a2}  '
      f'Prec={prec_a2:.3f} Rec={rec_a2:.3f} F1={f1_a2:.3f}')

both = sum(1 for r in out_rows if r['nlos_cn0'] and r['nlos_dd'])
print(f'\n  CN0 AND DD agreement: {both} ({100*both/n_total:.1f}% of all, '
      f'{100*both/max(n_cn0,1):.0f}% of CN0-flagged)')

with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
    w.writeheader(); w.writerows(out_rows)
print(f'\nSaved CSV: {args.out_csv}  ({len(out_rows)} rows)')

def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

elevs    = np.array([float(r['elev_deg'])    for r in out_rows])
cn0s     = np.array([float(r['cn0'])         for r in out_rows])
deficits = np.array([float(r['cn0_deficit']) for r in out_rows])
is_lidar = np.array([r['nlos_lidar']         for r in out_rows])
is_cn0   = np.array([r['nlos_cn0']           for r in out_rows])
is_dd    = np.array([r['nlos_dd']            for r in out_rows])

imgs = {}

fig, ax = plt.subplots(figsize=(9, 5))
ax.scatter(elevs[is_lidar==0], cn0s[is_lidar==0], s=4, alpha=0.3,
           color='#2196F3', label=f'LiDAR LOS (n={int((is_lidar==0).sum())})')
ax.scatter(elevs[is_lidar==1], cn0s[is_lidar==1], s=4, alpha=0.5,
           color='#F44336', label=f'LiDAR NLOS (n={int((is_lidar==1).sum())})')
if len(bin_centers):
    ax.plot(bin_centers, bin_medians, 'k-', lw=2.5, label='Expected C/N0 (median)')
    ax.fill_between(bin_centers, bin_p25, bin_p75, alpha=0.15, color='k', label='IQR band')
ax.set_xlabel('Elevation (deg)'); ax.set_ylabel('C/N0 (dB-Hz)')
ax.set_title('GPS L1 C/N0 vs Elevation - colored by LiDAR NLOS label')
ax.legend(markerscale=3, fontsize=9); ax.grid(True, alpha=0.3)
imgs['scatter'] = fig_to_b64(fig)

fig, ax = plt.subplots(figsize=(9, 4.5))
bins = np.linspace(-20, 30, 60)
ax.hist(deficits[is_lidar==0], bins=bins, alpha=0.6, color='#2196F3',
        label=f'LiDAR LOS  (n={(is_lidar==0).sum()})', density=True)
ax.hist(deficits[is_lidar==1], bins=bins, alpha=0.6, color='#F44336',
        label=f'LiDAR NLOS (n={(is_lidar==1).sum()})', density=True)
ax.axvline(args.cn0_thresh, color='k', ls='--', lw=1.5,
           label=f'Threshold = {args.cn0_thresh} dB-Hz')
ax.set_xlabel('C/N0 deficit (dB-Hz)  [expected - actual]')
ax.set_ylabel('Density')
ax.set_title('C/N0 Deficit Distribution - LOS vs NLOS')
ax.legend(); ax.grid(True, alpha=0.3)
imgs['hist'] = fig_to_b64(fig)

fig, axes = plt.subplots(1, 3, figsize=(11, 4), sharey=False)
methods = ['CN0', 'DD (5m)', 'Any-2']
metrics_data = {
    'Precision': [prec_cl, prec_dl, prec_a2],
    'Recall':    [rec_cl,  rec_dl,  rec_a2],
    'F1':        [f1_cl,   f1_dl,   f1_a2],
}
colors_m = ['#FF9800', '#2196F3', '#9C27B0']
for ax, (mname, vals) in zip(axes, metrics_data.items()):
    bars = ax.bar(methods, vals, color=colors_m, alpha=0.85, edgecolor='white')
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x()+bar.get_width()/2, v+0.01, f'{v:.3f}',
                ha='center', va='bottom', fontsize=9)
    ax.set_ylim(0, 1.1); ax.set_title(mname); ax.grid(axis='y', alpha=0.3)
    ax.set_ylabel(mname)
fig.suptitle('NLOS Detection vs LiDAR Ground Truth', fontsize=12)
plt.tight_layout()
imgs['prec_rec'] = fig_to_b64(fig)

fig, ax = plt.subplots(figsize=(10, 4))
only_cn0 = sum(1 for r in out_rows if r['nlos_cn0'] and not r['nlos_dd'] and not r['nlos_lidar'])
only_dd  = sum(1 for r in out_rows if not r['nlos_cn0'] and r['nlos_dd'] and not r['nlos_lidar'])
only_lid = sum(1 for r in out_rows if not r['nlos_cn0'] and not r['nlos_dd'] and r['nlos_lidar'])
cn0_dd   = sum(1 for r in out_rows if r['nlos_cn0'] and r['nlos_dd'] and not r['nlos_lidar'])
cn0_lid  = sum(1 for r in out_rows if r['nlos_cn0'] and not r['nlos_dd'] and r['nlos_lidar'])
dd_lid   = sum(1 for r in out_rows if not r['nlos_cn0'] and r['nlos_dd'] and r['nlos_lidar'])
all3     = sum(1 for r in out_rows if r['nlos_cn0'] and r['nlos_dd'] and r['nlos_lidar'])
labels_v = ['CN0 only','DD only','LiDAR only','CN0+DD','CN0+LiDAR','DD+LiDAR','All 3']
values_v = [only_cn0, only_dd, only_lid, cn0_dd, cn0_lid, dd_lid, all3]
colors_v = ['#FF9800','#2196F3','#4CAF50','#FF5722','#9C27B0','#00BCD4','#F44336']
bars = ax.bar(labels_v, values_v, color=colors_v, alpha=0.85, edgecolor='white')
for bar, v in zip(bars, values_v):
    ax.text(bar.get_x()+bar.get_width()/2, v+5, str(v), ha='center', va='bottom', fontsize=9)
ax.set_ylabel('Number of satellite observations')
ax.set_title('3-Way NLOS Overlap (CN0 / DD / LiDAR)')
ax.grid(axis='y', alpha=0.3)
imgs['venn'] = fig_to_b64(fig)

try:
    with open(args.dd_labels) as f:
        dd_full = {(round(float(r['rinex_t']),3), r['sat_id']): float(r['dd_resid'])
                   for r in csv.DictReader(f) if r['sys']=='G'}
    dr_list, def_list, lbl_list = [], [], []
    for row in out_rows:
        key = (round(float(row['rinex_t']),3), row['sat_id'])
        if key in dd_full:
            dr_list.append(dd_full[key])
            def_list.append(float(row['cn0_deficit']))
            lbl_list.append(row['nlos_lidar'])
    dr_arr  = np.array(dr_list)
    def_arr = np.array(def_list)
    lbl_arr = np.array(lbl_list)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(dr_arr[lbl_arr==0], def_arr[lbl_arr==0], s=4, alpha=0.3,
               color='#2196F3', label='LiDAR LOS')
    ax.scatter(dr_arr[lbl_arr==1], def_arr[lbl_arr==1], s=4, alpha=0.5,
               color='#F44336', label='LiDAR NLOS')
    ax.axhline(args.cn0_thresh, color='orange', ls='--', lw=1.2,
               label=f'CN0 thresh {args.cn0_thresh} dB')
    ax.axvline(5.0, color='purple', ls='--', lw=1.2, label='DD thresh 5m')
    ax.set_xlim(-60, 150); ax.set_ylim(-20, 30)
    ax.set_xlabel('DD residual (m)'); ax.set_ylabel('C/N0 deficit (dB-Hz)')
    ax.set_title('DD residual vs C/N0 deficit - feature space')
    ax.legend(markerscale=3, fontsize=9); ax.grid(True, alpha=0.3)
    imgs['feature_space'] = fig_to_b64(fig)
except Exception as e:
    print(f'  [WARN] feature-space plot skipped: {e}')

def pct(n): return f'{100*n/n_total:.1f}%' if n_total else '0%'

fs_img = ('\n<h2>5. Feature Space: DD residual vs C/N0 deficit</h2>\n<img src="data:image/png;base64,' +
          imgs['feature_space'] + '">' if 'feature_space' in imgs else '')

html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Step 8a - C/N0 NLOS Detection</title>
<style>
body{{font-family:sans-serif;margin:20px;background:#f4f6fb;color:#333}}
h1{{color:#1a237e;border-bottom:3px solid #3f51b5;padding-bottom:8px}}
h2{{color:#283593;margin-top:26px}}
img{{max-width:100%;border:1px solid #ddd;border-radius:6px;margin:8px 0}}
table{{border-collapse:collapse;width:100%}}
th{{background:#3f51b5;color:white;padding:8px 14px;text-align:left}}
td{{padding:7px 14px;border-bottom:1px solid #e8e8e8}}
tr:nth-child(even){{background:#f5f7ff}}
.key{{background:#e8f5e9;padding:14px;border-left:4px solid #43a047;margin:12px 0;border-radius:4px}}
.note{{background:#e3f2fd;padding:12px;border-left:4px solid #1976d2;margin:10px 0;font-size:.93em;border-radius:4px}}
</style></head><body>
<h1>Step 8a - C/N0 NLOS Detection (GPS L1 S1C)</h1>
<p>UrbanNav HK Medium-Urban-1 | {n_total} GPS L1 obs | Elev cutoff: {args.min_elev}deg | Deficit threshold: {args.cn0_thresh} dB-Hz</p>
<div class="note">
<b>Method:</b> cn0_deficit = expected_C/N0(elev) - actual_C/N0. Expected from 5-deg binned medians.
Deficit &gt; {args.cn0_thresh} dB-Hz flagged as NLOS.
</div>
<div class="key">
<b>Summary</b><br>
CN0-flagged: <b>{n_cn0}</b> ({pct(n_cn0)}) | DD-flagged: {n_dd} ({pct(n_dd)}) |
LiDAR: {n_lidar} ({pct(n_lidar)}) | Any-2 consensus: <b>{n_any2}</b> ({pct(n_any2)})<br><br>
<table style="width:auto">
<tr><th>Method</th><th>Precision</th><th>Recall</th><th>F1</th></tr>
<tr><td>CN0 (deficit&gt;{args.cn0_thresh}dB)</td><td>{prec_cl:.3f}</td><td>{rec_cl:.3f}</td><td>{f1_cl:.3f}</td></tr>
<tr><td>DD (resid&gt;5m)</td><td>{prec_dl:.3f}</td><td>{rec_dl:.3f}</td><td>{f1_dl:.3f}</td></tr>
<tr><td>Any-2 of 3</td><td>{prec_a2:.3f}</td><td>{rec_a2:.3f}</td><td>{f1_a2:.3f}</td></tr>
</table>
</div>
<h2>1. C/N0 vs Elevation</h2>
<img src="data:image/png;base64,{imgs['scatter']}">
<h2>2. C/N0 Deficit Distribution</h2>
<img src="data:image/png;base64,{imgs['hist']}">
<h2>3. Precision / Recall / F1</h2>
<img src="data:image/png;base64,{imgs['prec_rec']}">
<h2>4. 3-Way NLOS Overlap</h2>
<img src="data:image/png;base64,{imgs['venn']}">
{fs_img}
</body></html>"""

with open(args.out_html, 'w') as f:
    f.write(html)
print(f'Saved HTML: {args.out_html}')
