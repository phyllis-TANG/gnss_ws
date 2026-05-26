#!/usr/bin/env python3
"""
lidar_step4c_topdown_viz.py
Bird's-eye view of the LiDAR point cloud map overlaid with ray casting
results (LOS / 1-bounce / 2-bounce), coloured by bounce type.

Inputs:
  - urbannav_map.pcd          (Step2: LiDAR map in ENU)
  - novatel_trajectory.csv    (Step2: reference trajectory, defines ENU origin)
  - epoch_sat_azel.csv        (Step3: receiver positions + satellite angles)
  - lidar_nlos_prediction_2b.csv  (Step4: 2-bounce ray casting output)

Output:
  - /root/topdown_viz.html    (embedded PNG, ~2–4 MB)

Usage:
  python3 lidar_step4c_topdown_viz.py \
    --pcd   /root/urbannav_map.pcd \
    --traj  /root/novatel_trajectory.csv \
    --azel  /root/epoch_sat_azel.csv \
    --nlos  /root/lidar_nlos_prediction_2b.csv \
    --out   /root/topdown_viz.html
"""

import argparse, base64, csv, io, math, struct, time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser()
ap.add_argument('--pcd',   default='/root/urbannav_map.pcd')
ap.add_argument('--traj',  default='/root/novatel_trajectory.csv')
ap.add_argument('--azel',  default='/root/epoch_sat_azel.csv')
ap.add_argument('--nlos',  default='/root/lidar_nlos_prediction_2b.csv')
ap.add_argument('--out',   default='/root/topdown_viz.html')
ap.add_argument('--ray_every', type=int, default=6,
                help='Draw every Nth ray segment (default 6, to reduce clutter)')
ap.add_argument('--map_res', type=float, default=0.5,
                help='Top-down density map resolution in metres (default 0.5)')
args = ap.parse_args()

# ── read PCD ─────────────────────────────────────────────────────────
print(f'Reading point cloud: {args.pcd}')
t0 = time.time()

def read_pcd_xy(path):
    """Read PCD, return only X/Y columns (top-down projection)."""
    with open(path, 'rb') as f:
        raw = f.read()
    lines = raw.split(b'\n')
    header = {}
    data_start = 0
    for i, line in enumerate(lines):
        s = line.strip().decode('latin-1', errors='replace')
        if s.startswith('DATA'):
            header['DATA'] = s.split()[1]
            data_start = i + 1
            break
        parts = s.split()
        if len(parts) >= 2:
            header[parts[0]] = parts[1:]

    n_pts    = int(header.get('POINTS', [0])[0])
    fields   = header.get('FIELDS', [])
    sizes    = [int(s) for s in header.get('SIZE', [])]
    types    = header.get('TYPE', [])
    data_fmt = header.get('DATA', 'ascii')

    xi = fields.index('x')
    yi = fields.index('y')
    zi = fields.index('z')

    if data_fmt == 'ascii':
        pts = []
        for line in lines[data_start:]:
            p = line.strip().split()
            if len(p) > max(xi, yi, zi):
                try:
                    pts.append([float(p[xi]), float(p[yi]), float(p[zi])])
                except ValueError:
                    pass
        return np.array(pts, dtype=np.float32)

    # binary
    byte_offset = sum(len(l) + 1 for l in lines[:data_start])
    data_bytes  = raw[byte_offset:]
    point_step  = sum(sizes)
    fmt_map     = {'F': 'f', 'I': 'i', 'U': 'u'}
    dtype       = np.dtype([(f, f'{fmt_map.get(t,"u")}{s}')
                             for f, s, t in zip(fields, sizes, types)])
    arr = np.frombuffer(data_bytes[:n_pts * point_step], dtype=dtype)
    return np.column_stack([arr['x'].astype(np.float32),
                             arr['y'].astype(np.float32),
                             arr['z'].astype(np.float32)])

cloud = read_pcd_xy(args.pcd)
print(f'  {len(cloud):,} points in {time.time()-t0:.1f}s')

# ── ENU origin from trajectory ────────────────────────────────────────
def lla_to_ecef(lat_deg, lon_deg, alt_m):
    a, e2 = 6378137.0, 6.69437999014e-3
    lat = math.radians(lat_deg); lon = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    return np.array([(N+alt_m)*math.cos(lat)*math.cos(lon),
                     (N+alt_m)*math.cos(lat)*math.sin(lon),
                     (N*(1-e2)+alt_m)*math.sin(lat)])

def ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon):
    lat = math.radians(ref_lat); lon = math.radians(ref_lon)
    R = np.array([[-math.sin(lon), math.cos(lon), 0],
                  [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
                  [ math.cos(lat)*math.cos(lon),  math.cos(lat)*math.sin(lon), math.sin(lat)]])
    return R @ (ecef - ref_ecef)

with open(args.traj) as f:
    r0 = next(csv.DictReader(f))
ref_lat = float(r0['lat']); ref_lon = float(r0['lon']); ref_alt = float(r0['alt_m'])
ref_ecef = lla_to_ecef(ref_lat, ref_lon, ref_alt)
print(f'ENU origin: lat={ref_lat:.5f} lon={ref_lon:.5f}')

# ── build 2D density map of the point cloud ───────────────────────────
print('Building top-down density map...')
min_z = 1.0   # filter ground returns
z_mask = cloud[:, 2] > min_z
xy = cloud[z_mask, :2]   # East, North

res = args.map_res
xmin, xmax = xy[:, 0].min(), xy[:, 0].max()
ymin, ymax = xy[:, 1].min(), xy[:, 1].max()
nx = int((xmax - xmin) / res) + 1
ny = int((ymax - ymin) / res) + 1
grid = np.zeros((ny, nx), dtype=np.uint16)

ix = np.clip(((xy[:, 0] - xmin) / res).astype(int), 0, nx-1)
iy = np.clip(((xy[:, 1] - ymin) / res).astype(int), 0, ny-1)
np.add.at(grid, (iy, ix), 1)
print(f'  density grid: {nx} x {ny} cells')

# ── load azel → receiver ENU positions ──────────────────────────────
print('Loading azel data...')
azel_rows = []
with open(args.azel) as f:
    for row in csv.DictReader(f):
        rx_ecef = lla_to_ecef(float(row['rx_lat']), float(row['rx_lon']),
                               float(row['rx_alt']))
        rx_enu  = ecef_to_enu(rx_ecef, ref_ecef, ref_lat, ref_lon)
        azel_rows.append({
            'unix_t': row['unix_t'],
            'sat_id': row['sat_id'],
            'rx_e':   rx_enu[0],
            'rx_n':   rx_enu[1],
            'azim':   float(row['azimuth_deg']),
            'elev':   float(row['elevation_deg']),
        })
azel_dict = {(r['unix_t'], r['sat_id']): r for r in azel_rows}

# collect unique receiver positions for trajectory line
traj_pts = {}
for r in azel_rows:
    traj_pts[r['unix_t']] = (r['rx_e'], r['rx_n'])
traj_sorted = [v for _, v in sorted(traj_pts.items())]
traj_e = [p[0] for p in traj_sorted]
traj_n = [p[1] for p in traj_sorted]

# ── load 2b NLOS results ─────────────────────────────────────────────
print('Loading 2b NLOS results...')
nlos_rows = []
with open(args.nlos) as f:
    for row in csv.DictReader(f):
        nlos_rows.append({
            'unix_t':  row['unix_t'],
            'sat_id':  row['sat_id'],
            'n_b':     int(row['n_bounces']),
            'dist1':   float(row['hit_dist_m']),
        })

# ── draw figure ──────────────────────────────────────────────────────
print('Drawing figure...')
fig, ax = plt.subplots(figsize=(14, 14))
fig.patch.set_facecolor('#1a1a2e')
ax.set_facecolor('#1a1a2e')

# point cloud density — log scale for contrast
log_grid = np.log1p(grid.astype(np.float32))
extent = [xmin, xmax, ymin, ymax]
ax.imshow(log_grid, origin='lower', extent=extent,
          cmap='Blues', alpha=0.85, aspect='equal',
          vmin=0, vmax=log_grid.max() * 0.7)

# receiver trajectory
ax.plot(traj_e, traj_n, '-', color='cyan', lw=1.2, alpha=0.7, zorder=3)
ax.plot(traj_e[0],  traj_n[0],  'o', color='lime', ms=8, zorder=5, label='Start')
ax.plot(traj_e[-1], traj_n[-1], 's', color='yellow', ms=8, zorder=5, label='End')

# ray segments coloured by bounce type
colors = {0: ('#00ff88', 0.07),   # LOS   — faint green
          1: ('#ff8800', 0.25),   # 1b    — orange
          2: ('#ff2222', 0.40)}   # 2b    — red

for i, nr in enumerate(nlos_rows):
    if i % args.ray_every != 0:
        continue
    key = (nr['unix_t'], nr['sat_id'])
    ar = azel_dict.get(key)
    if ar is None:
        continue
    n_b   = nr['n_b']
    dist  = nr['dist1'] if n_b > 0 else 15.0   # LOS: draw 15m stub
    az_r  = math.radians(ar['azim'])
    el_r  = math.radians(ar['elev'])
    cos_el = math.cos(el_r)
    de = math.sin(az_r) * cos_el
    dn = math.cos(az_r) * cos_el
    col, alpha = colors.get(n_b, ('#ffffff', 0.1))
    ex = ar['rx_e'] + de * dist
    en = ar['rx_n'] + dn * dist
    ax.plot([ar['rx_e'], ex], [ar['rx_n'], en],
            '-', color=col, alpha=alpha, lw=0.6, zorder=2)

# legend
legend_elements = [
    Line2D([0],[0], color='cyan',    lw=1.5, label='Receiver trajectory'),
    Line2D([0],[0], color='#00ff88', lw=1.5, alpha=0.6, label='LOS ray stub'),
    Line2D([0],[0], color='#ff8800', lw=1.5, alpha=0.8, label='1-bounce ray'),
    Line2D([0],[0], color='#ff2222', lw=1.5, alpha=0.9, label='2-bounce ray'),
]
ax.legend(handles=legend_elements, loc='upper right', fontsize=10,
          framealpha=0.7, facecolor='#1a1a2e', labelcolor='white')

# counts summary
n_los = sum(1 for r in nlos_rows if r['n_b'] == 0)
n_1b  = sum(1 for r in nlos_rows if r['n_b'] == 1)
n_2b  = sum(1 for r in nlos_rows if r['n_b'] == 2)
ax.set_title(
    f'UrbanNav HK — LiDAR Map + Ray Casting (top-down)\n'
    f'LOS: {n_los}  1-bounce: {n_1b}  2-bounce: {n_2b}  '
    f'(every {args.ray_every}th ray shown)',
    color='white', fontsize=12, pad=10)
ax.set_xlabel('East (m)', color='white'); ax.set_ylabel('North (m)', color='white')
ax.tick_params(colors='white')
for sp in ax.spines.values():
    sp.set_edgecolor('#555')
plt.tight_layout()

print('Encoding figure...')
buf = io.BytesIO()
fig.savefig(buf, format='png', dpi=130, bbox_inches='tight',
            facecolor=fig.get_facecolor())
plt.close(fig)
img_b64 = base64.b64encode(buf.getvalue()).decode()

# ── generate HTML ────────────────────────────────────────────────────
html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Top-Down LiDAR + Ray Casting</title>
<style>
body{{font-family:sans-serif;margin:20px;background:#111;color:#eee}}
h1{{color:#ccc}} p{{color:#aaa}}
img{{max-width:100%;border:1px solid #444;border-radius:6px}}
.key{{background:#1e2a3a;padding:12px;border-left:4px solid #4a90d9;margin:10px 0}}
</style></head><body>
<h1>UrbanNav HK — LiDAR Map + 2-Bounce Ray Casting (Bird's Eye View)</h1>
<div class="key">
  Point cloud: {len(cloud):,} pts (Z&gt;1m kept) &nbsp;|&nbsp;
  ENU grid resolution: {res}m &nbsp;|&nbsp;
  Rays shown: every {args.ray_every}th<br>
  LOS: {n_los} &nbsp;|&nbsp; 1-bounce NLOS: {n_1b} &nbsp;|&nbsp;
  2-bounce NLOS: {n_2b}
</div>
<img src="data:image/png;base64,{img_b64}">
<p>Blue map = LiDAR point cloud (log density, top-down). &nbsp;
Cyan = receiver trajectory. &nbsp;
<span style="color:#00ff88">Green stubs</span> = LOS sats. &nbsp;
<span style="color:#ff8800">Orange</span> = 1-bounce. &nbsp;
<span style="color:#ff2222">Red</span> = 2-bounce.</p>
</body></html>"""

with open(args.out, 'w') as f:
    f.write(html)
print(f'Saved: {args.out}')
