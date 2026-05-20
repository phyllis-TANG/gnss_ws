#!/usr/bin/env python3
"""
lidar_step7_spp_correction.py
用 LiDAR NLOS 标签和 ΔL 估计对 SPP 伪距进行修正，对比三种模式的定位精度。

三种模式：
  baseline   — 所有卫星，不做任何修正（复现原始 SPP 误差）
  exclusion  — 剔除 LiDAR 判定为 NLOS 的卫星
  correction — 保留 NLOS 卫星，伪距减去 ΔL，权重按平面性降权

核心算法参考：
  [1] Kaplan & Hegarty (2006), Understanding GPS, Ch.7 — WLS SPP
  [2] Groves (2013), Principles of GNSS, Ch.9  — 仰角加权
  [3] Wen et al. (2019), NAVIGATION doi:10.1002/navi.335 — LiDAR NLOS 伪距修正
  [4] IS-GPS-200 (ICD) — 卫星钟差公式 & Sagnac 效应修正

运行（容器内）：
  python3 lidar_step7_spp_correction.py \\
    --obs  /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
    --nav  /root/urbannav_gnss/hksc137c.21n \\
    --nlos /root/lidar_nlos_prediction.csv \\
    --refl /root/lidar_reflection_model.csv \\
    --gt   /root/urbannav_gt.txt \\
    --out_csv  /root/spp_correction_results.csv \\
    --out_html /root/spp_correction_report.html
"""

import argparse, csv, math, os, sys, json, datetime
import numpy as np

# ── rinex_utils 路径（与 Step 3 一致） ────────────────────────────────
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
OMEGA_E      = 7.2921151467e-5  # 地球自转角速度 (rad/s)

# ── 参数 ──────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument('--obs',  default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav',  default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nlos', default='/root/lidar_nlos_prediction.csv')
ap.add_argument('--refl', default='/root/lidar_reflection_model.csv')
ap.add_argument('--gt',   default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',  default='/root/spp_correction_results.csv')
ap.add_argument('--out_html', default='/root/spp_correction_report.html')
ap.add_argument('--min_elev', type=float, default=10.0,
                help='最小仰角截止（度），低于此值跳过（默认10°）')
ap.add_argument('--gt_tol',   type=float, default=10.0,
                help='GT 时间匹配容差（秒，默认10s）')
ap.add_argument('--nlos_weight_factor', type=float, default=0.1,
                help='NLOS 修正模式的权重折减因子（默认 0.1）')
args = ap.parse_args()

# ── 坐标工具 ──────────────────────────────────────────────────────────
def ecef_to_enu(ecef, ref_ecef, ref_lat_deg, ref_lon_deg):
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    R = np.array([
        [-math.sin(lon),                 math.cos(lon),                0],
        [-math.sin(lat)*math.cos(lon),  -math.sin(lat)*math.sin(lon),  math.cos(lat)],
        [ math.cos(lat)*math.cos(lon),   math.cos(lat)*math.sin(lon),  math.sin(lat)],
    ])
    return R @ (ecef - ref_ecef)

def sagnac_correct(sat_ecef, travel_time_s):
    """
    Sagnac 效应修正（地球自转修正）[4] IS-GPS-200。
    信号传播期间地球旋转，等效于卫星位置沿 Z 轴旋转。
    """
    theta = OMEGA_E * travel_time_s
    c, s  = math.cos(theta), math.sin(theta)
    R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return R @ sat_ecef

def tropo_delay(elev_deg):
    """
    简化对流层延迟（Hopfield 简化模型），单位：米。
    zenith delay ≈ 2.3m，按仰角映射。
    """
    el = max(elev_deg, 3.0)
    return 2.3 / math.sin(math.radians(el) + 0.017)

# ── WLS SPP 求解器 ─────────────────────────────────────────────────────
def wls_spp(sats_info, x0_ecef, min_sats=4, max_iter=10):
    """
    迭代加权最小二乘 SPP，同时估计位置（XYZ）和接收机钟差。[1][2]

    sats_info: list of dict，每条包含：
      sat_ecef   : 卫星 ECEF（Sagnac 已修正）
      psr_corr   : 卫星钟差修正后的伪距（含 ΔL 修正 / 对流层修正）
      weight     : 权重（0 = 排除）
      elevation  : 仰角（度），用于 PDOP 计算

    返回 (pos_ecef[3], clk_m, pdop) 或 (None, None, None)。
    """
    active = [s for s in sats_info if s['weight'] > 0]
    if len(active) < min_sats:
        return None, None, None

    x = np.array(list(x0_ecef) + [0.0], dtype=float)  # [X, Y, Z, c·δt_r]

    for _ in range(max_iter):
        H_list, dp_list, w_list = [], [], []
        for s in active:
            diff = x[:3] - np.array(s['sat_ecef'])
            r    = np.linalg.norm(diff)
            if r < 1e4:          # 不合理距离（卫星不在 LEO 以下）
                continue
            e = diff / r         # 接收机→卫星反向单位向量
            H_list.append([e[0], e[1], e[2], 1.0])
            dp_list.append(s['psr_corr'] - r - x[3])
            w_list.append(s['weight'])

        if len(H_list) < min_sats:
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

    # PDOP（几何精度因子）[1]
    try:
        Q = np.linalg.inv(H.T @ H)
        pdop = math.sqrt(max(Q[0,0] + Q[1,1] + Q[2,2], 0))
    except Exception:
        pdop = float('nan')

    return x[:3], x[3], pdop

# ── 加载 NLOS 标签 ─────────────────────────────────────────────────────
print('加载 LiDAR NLOS 标签...')
nlos_dict = {}   # (unix_t_str, sat_id) → 1/0
with open(args.nlos) as f:
    for row in csv.DictReader(f):
        nlos_dict[(row['unix_t'], row['sat_id'])] = int(row['lidar_nlos'])
print(f'  {len(nlos_dict)} 条')

# ── 加载 ΔL 和平面性 ───────────────────────────────────────────────────
print('加载反射面 ΔL...')
refl_dict = {}   # (unix_t_str, sat_id) → (delta_l, planarity)
with open(args.refl) as f:
    for row in csv.DictReader(f):
        dl   = float(row['delta_L_m'])     if row['delta_L_m']  else 0.0
        plan = float(row['planarity'])     if row['planarity']  else 0.3
        refl_dict[(row['unix_t'], row['sat_id'])] = (dl, plan)
print(f'  {len(refl_dict)} 条')

# ── 加载地面真值 ───────────────────────────────────────────────────────
# 文件格式（UrbanNav GT）：空格分隔，前两行为表头
# 列：UTCTime(unix秒) Week GPSTime Lat_D Lat_M Lat_S Lon_D Lon_M Lon_S H-Ell ...
# 示例：1621218775.00 2158.00000 95593.00  22 18 04.31949  114 10 44.60559  3.472 ...
print(f'加载地面真值: {args.gt}')
gt_times, gt_lats, gt_lons, gt_alts = [], [], [], []

def dms_to_deg(d, m, s):
    return float(d) + float(m) / 60.0 + float(s) / 3600.0

with open(args.gt) as f:
    lines = f.readlines()

for line in lines:
    line = line.strip()
    if not line or line.startswith('UTC') or line.startswith('('):
        continue          # 跳过两行文字表头
    parts = line.split()
    if len(parts) < 10:
        continue
    try:
        utc_t = float(parts[0])    # Unix 时间戳（秒）
        lat   = dms_to_deg(parts[3], parts[4], parts[5])
        lon   = dms_to_deg(parts[6], parts[7], parts[8])
        alt   = float(parts[9])
        if utc_t < 1e9:            # 不合理的时间戳（表头残留）
            continue
        gt_times.append(utc_t)
        gt_lats.append(lat)
        gt_lons.append(lon)
        gt_alts.append(alt)
        t = utc_t   # 占位，保持后续代码结构一致
        if t is None:
            continue
    except (ValueError, IndexError):
        continue

gt_times = np.array(gt_times)
gt_lats  = np.array(gt_lats)
gt_lons  = np.array(gt_lons)
gt_alts  = np.array(gt_alts)
order = np.argsort(gt_times)
gt_times, gt_lats, gt_lons, gt_alts = (gt_times[order], gt_lats[order],
                                        gt_lons[order], gt_alts[order])
print(f'  {len(gt_times)} 个 GT 点，UTC {gt_times[0]:.1f} ~ {gt_times[-1]:.1f}')

def match_gt(utc_t):
    """最近邻时间匹配，超出容差返回 None"""
    idx = int(np.searchsorted(gt_times, utc_t))
    idx = min(max(idx, 0), len(gt_times)-1)
    if idx > 0 and abs(gt_times[idx-1]-utc_t) < abs(gt_times[idx]-utc_t):
        idx -= 1
    if abs(gt_times[idx] - utc_t) > args.gt_tol:
        return None
    return gt_lats[idx], gt_lons[idx], gt_alts[idx]

# ── 读星历 ─────────────────────────────────────────────────────────────
print('读取导航电文...')
ephem_dict = read_rinex_nav(args.nav)
print(f'  {len(ephem_dict)} 颗卫星')

# ── 读观测文件 ─────────────────────────────────────────────────────────
print('读取观测文件...')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} 个历元')

# ── 初始化位置（香港 UrbanNav 近似，用于 WLS 线性化） ─────────────────
x0_ecef = np.array(llh_to_ecef(22.3198, 114.2095, 20.0))

# ── 主循环 ─────────────────────────────────────────────────────────────
print(f'\n运行 SPP（三模式，min_elev={args.min_elev}°）...')

results = []    # 每历元的结果
no_gt   = 0

for ep_i, epoch in enumerate(obs_epochs):
    rinex_unix_t = epoch.time_unix
    utc_t        = rinex_unix_t - LEAP_SECONDS

    # GT 匹配
    gt_match = match_gt(utc_t)
    if gt_match is None:
        no_gt += 1
        continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    # ── 构建每颗卫星的观测数据 ─────────────────────────────────────────
    sat_data = []   # list of dict

    for obs in epoch.obs_list:
        sat_id = obs.sat_id

        # 只取有星历的 GPS 卫星
        eph_list = ephem_dict.get(sat_id, [])
        eph = find_closest_ephem(eph_list, rinex_unix_t)
        if eph is None:
            continue

        sat_ecef_raw, dt_sv = compute_sat_position(eph, rinex_unix_t)
        if sat_ecef_raw is None:
            continue

        # 伪距
        psr_raw = 0.0
        for key in ['C1C', 'C1P', 'C1X', 'C2C', 'C2P']:
            if key in obs.pseudorange and obs.pseudorange[key] > 0:
                psr_raw = obs.pseudorange[key]
                break
        if psr_raw <= 0:
            continue

        # 卫星钟差修正后的伪距 [4]
        psr_clk = psr_raw - C_LIGHT * dt_sv

        # 信号传播时间 → Sagnac 修正 [4]
        travel_t   = psr_raw / C_LIGHT
        sat_ecef_s = sagnac_correct(np.array(sat_ecef_raw), travel_t)

        # 计算仰角（用于截止角过滤和权重）
        diff_rx  = np.array(sat_ecef_s) - x0_ecef
        r_approx = np.linalg.norm(diff_rx)
        gt_lat_r  = math.radians(gt_lat)
        gt_lon_r  = math.radians(gt_lon)
        up = np.array([
            math.cos(gt_lat_r)*math.cos(gt_lon_r),
            math.cos(gt_lat_r)*math.sin(gt_lon_r),
            math.sin(gt_lat_r)])
        elev_approx = math.degrees(math.asin(
            np.dot(diff_rx / r_approx, up)))

        if elev_approx < args.min_elev:
            continue

        # 对流层修正（减去延迟）
        psr_tropo = psr_clk - tropo_delay(elev_approx)

        # 仰角加权 [2]
        w_base = math.sin(math.radians(elev_approx)) ** 2

        # NLOS 标签 & ΔL
        key_nt = (f'{rinex_unix_t:.3f}', sat_id)
        is_nlos = nlos_dict.get(key_nt, 0)
        delta_l, planarity = refl_dict.get(key_nt, (0.0, 0.5))

        sat_data.append({
            'sat_id':    sat_id,
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

    # ── 三种模式 ──────────────────────────────────────────────────────
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
            else:  # correction
                if s['is_nlos']:
                    # 伪距减去 ΔL，权重按平面性降权 [3]
                    psr = s['psr_tropo'] - s['delta_l']
                    w   = s['w_base'] * s['planarity'] * args.nlos_weight_factor
                else:
                    w   = s['w_base']
                    psr = s['psr_tropo']
            sats_info.append({
                'sat_ecef':  s['sat_ecef'],
                'psr_corr':  psr,
                'weight':    w,
                'elevation': s['elev'],
            })

        pos, clk, pdop = wls_spp(sats_info, x0_ecef)

        if pos is None:
            mode_results[mode] = None
            continue

        # 误差：ECEF 差 → ENU → 2D / 3D
        err_ecef = pos - gt_ecef
        err_enu  = ecef_to_enu(pos, gt_ecef, gt_lat, gt_lon)
        err_h    = math.sqrt(err_enu[0]**2 + err_enu[1]**2)   # 水平误差
        err_v    = abs(err_enu[2])                              # 垂直误差
        err_3d   = math.sqrt(err_h**2 + err_v**2)

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
        'utc_t':         round(utc_t, 3),
        'n_total':       n_total,
        'n_nlos':        n_nlos,
        **{f'{m}_{k}': (mode_results[m][k] if mode_results[m] else None)
           for m in ('baseline','exclusion','correction')
           for k in ('err_h','err_v','err_3d','pdop','n_used')},
    })

    if (ep_i+1) % 50 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} 历元...', end='', flush=True)

print(f'\n完成：{len(results)} 个有效历元（{no_gt} 个超出 GT 时间范围）')

# ── 写 CSV ────────────────────────────────────────────────────────────
COLS = ['utc_t','n_total','n_nlos',
        'baseline_err_h','baseline_err_v','baseline_err_3d','baseline_pdop','baseline_n_used',
        'exclusion_err_h','exclusion_err_v','exclusion_err_3d','exclusion_pdop','exclusion_n_used',
        'correction_err_h','correction_err_v','correction_err_3d','correction_pdop','correction_n_used']
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS, extrasaction='ignore')
    w.writeheader()
    w.writerows(results)
print(f'已保存 CSV：{args.out_csv}')

# ── 统计 ──────────────────────────────────────────────────────────────
def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return dict(n=0, mean=float('nan'), rms=float('nan'), p50=float('nan'), p95=float('nan'))
    a = np.array(v)
    return dict(
        n    = len(v),
        mean = float(np.mean(a)),
        rms  = float(np.sqrt(np.mean(a**2))),
        p50  = float(np.percentile(a, 50)),
        p95  = float(np.percentile(a, 95)),
    )

for mode in ('baseline', 'exclusion', 'correction'):
    eh = [r[f'{mode}_err_h'] for r in results]
    s  = stats(eh)
    print(f'  {mode:12s} 水平误差  n={s["n"]:4d}  '
          f'均值={s["mean"]:7.2f}m  RMS={s["rms"]:7.2f}m  '
          f'50th={s["p50"]:7.2f}m  95th={s["p95"]:7.2f}m')

# ── HTML 报告 ─────────────────────────────────────────────────────────
def cdf_data(vals, n_pts=200):
    v = sorted(x for x in vals if x is not None)
    if not v:
        return []
    mn, mx = v[0], min(v[-1], np.percentile(v, 99)*1.2)
    xs = np.linspace(mn, mx, n_pts)
    ys = [sum(1 for vi in v if vi <= x) / len(v) for x in xs]
    return [{'x': round(float(x),2), 'y': round(y,4)} for x,y in zip(xs,ys)]

def ts_data(results, key):
    return [{'x': r['utc_t'], 'y': r[key]} for r in results if r[key] is not None]

modes = ('baseline', 'exclusion', 'correction')
colors = {'baseline':'rgba(244,67,54,.7)', 'exclusion':'rgba(255,152,0,.8)',
          'correction':'rgba(33,150,243,.85)'}
labels_cn = {'baseline':'基准（无修正）','exclusion':'NLOS 排除','correction':'NLOS 修正（ΔL）'}

st = {m: stats([r[f'{m}_err_h'] for r in results]) for m in modes}

cdf_js    = {m: json.dumps(cdf_data([r[f'{m}_err_h'] for r in results])) for m in modes}
ts_js     = {m: json.dumps(ts_data(results, f'{m}_err_h'))               for m in modes}
ts_3d_js  = {m: json.dumps(ts_data(results, f'{m}_err_3d'))              for m in modes}

bar_labels = json.dumps(['均值 (m)', 'RMS (m)', '50th (m)', '95th (m)'])
bar_data   = {m: json.dumps([round(st[m][k],2) for k in ('mean','rms','p50','p95')]) for m in modes}

# NLOS 卫星数时间序列
nlos_ts_js = json.dumps([{'x': r['utc_t'], 'y': r['n_nlos']} for r in results])
total_ts_js = json.dumps([{'x': r['utc_t'], 'y': r['n_total']} for r in results])

html = f"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<title>Step 7 — SPP NLOS 修正对比报告</title>
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
  .card    {{ padding:16px; border-radius:8px; text-align:center; }}
  .card .val {{ font-size:1.8em; font-weight:bold; }}
  .card .lbl {{ font-size:.82em; color:#555; margin-top:4px; }}
  .base {{ background:#ffebee; }}  .excl {{ background:#fff3e0; }}
  .corr {{ background:#e3f2fd; }}
  .delta {{ font-size:.9em; font-weight:bold; color:#388e3c; }}
  table {{ border-collapse:collapse; width:100%; font-size:.9em; }}
  th {{ background:#3f51b5; color:#fff; padding:8px 12px; text-align:left; }}
  td {{ padding:7px 12px; border-bottom:1px solid #e8e8e8; }}
  tr:nth-child(even) {{ background:#f5f7ff; }}
  .better {{ color:#2e7d32; font-weight:bold; }}
  .worse  {{ color:#c62828; }}
  .ref {{ font-size:.8em; color:#666; line-height:1.9; }}
  .chart-box {{ padding:8px; }}
  .note {{ font-size:.82em; color:#777; margin:3px 0 10px; }}
  .formula {{ background:#f8f9fa; border-left:4px solid #3f51b5;
              padding:8px 14px; font-family:monospace; margin:8px 0; }}
</style>
</head>
<body>
<h1>Step 7 — LiDAR 辅助 SPP NLOS 修正对比</h1>
<p>UrbanNav Medium-Urban-1（香港 TST，2021-05-17）&nbsp;·&nbsp;
   有效历元：{len(results)} &nbsp;·&nbsp; 仰角截止：{args.min_elev}°</p>

<div class="section">
  <h2>方法</h2>
  <div class="formula">
    基准:   ρ̃_i = ρ_i − c·δt_s,i − T_i   （所有卫星，仰角加权）<br>
    排除:   剔除 LiDAR 判定的 NLOS 卫星<br>
    修正:   ρ̃_corrected = ρ̃_i − ΔL_i，权重 × planarity × {args.nlos_weight_factor}
  </div>
  <p class="ref">
    [1] Kaplan &amp; Hegarty (2006) — WLS SPP 迭代求解<br>
    [2] Groves (2013) — 仰角加权 w = sin²(el)<br>
    [3] Wen et al. (2019) NAVIGATION doi:10.1002/navi.335 — NLOS 伪距修正<br>
    [4] IS-GPS-200 — 卫星钟差 &amp; Sagnac 效应修正
  </p>
</div>

<div class="section">
  <h2>核心结果（水平误差）</h2>
  <div class="grid3">
    <div class="card base">
      <div class="val">{st['baseline']['mean']:.1f} m</div>
      <div class="lbl">基准均值</div>
      <div class="lbl">RMS {st['baseline']['rms']:.1f}m &nbsp; 95th {st['baseline']['p95']:.1f}m</div>
    </div>
    <div class="card excl">
      <div class="val">{st['exclusion']['mean']:.1f} m</div>
      <div class="lbl">NLOS 排除均值</div>
      <div class="lbl">RMS {st['exclusion']['rms']:.1f}m &nbsp; 95th {st['exclusion']['p95']:.1f}m</div>
      <div class="delta">
        {"↓{:.1f}m".format(st['baseline']['mean']-st['exclusion']['mean'])
         if st['exclusion']['mean'] < st['baseline']['mean']
         else "↑{:.1f}m".format(st['exclusion']['mean']-st['baseline']['mean'])}
      </div>
    </div>
    <div class="card corr">
      <div class="val">{st['correction']['mean']:.1f} m</div>
      <div class="lbl">NLOS 修正均值</div>
      <div class="lbl">RMS {st['correction']['rms']:.1f}m &nbsp; 95th {st['correction']['p95']:.1f}m</div>
      <div class="delta">
        {"↓{:.1f}m".format(st['baseline']['mean']-st['correction']['mean'])
         if st['correction']['mean'] < st['baseline']['mean']
         else "↑{:.1f}m".format(st['correction']['mean']-st['baseline']['mean'])}
      </div>
    </div>
  </div>

  <h3 style="margin-top:20px">详细统计表</h3>
  <table>
    <tr><th>模式</th><th>有效历元</th><th>均值 (m)</th><th>RMS (m)</th>
        <th>50th (m)</th><th>95th (m)</th><th>vs 基准</th></tr>
    {''.join(
      f'<tr><td>{labels_cn[m]}</td>'
      f'<td>{st[m]["n"]}</td>'
      f'<td>{st[m]["mean"]:.2f}</td>'
      f'<td>{st[m]["rms"]:.2f}</td>'
      f'<td>{st[m]["p50"]:.2f}</td>'
      f'<td>{st[m]["p95"]:.2f}</td>'
      f'<td class="{"better" if st[m]["mean"]<=st["baseline"]["mean"] else "worse"}">'
      f'{"−{:.2f}m".format(st["baseline"]["mean"]-st[m]["mean"]) if m!="baseline" else "—"}'
      f'</td></tr>'
      for m in modes
    )}
  </table>
</div>

<div class="section">
  <h2>可视化</h2>
  <div class="grid2">
    <div class="chart-box">
      <h3>水平误差 CDF（核心图）</h3>
      <p class="note">曲线越靠左越好。同一误差值下 CDF 越高代表更多历元达到该精度。</p>
      <canvas id="cCDF" height="260"></canvas>
    </div>
    <div class="chart-box">
      <h3>误差统计柱状图</h3>
      <canvas id="cBar" height="260"></canvas>
    </div>
  </div>
  <div class="chart-box" style="margin-top:16px">
    <h3>水平误差时间序列</h3>
    <p class="note">每个历元三种模式的水平误差对比</p>
    <canvas id="cTS" height="200"></canvas>
  </div>
  <div class="chart-box" style="margin-top:16px">
    <h3>每历元卫星数（总 / NLOS）</h3>
    <canvas id="cSatCnt" height="160"></canvas>
  </div>
</div>

<script>
const MODES = ['baseline','exclusion','correction'];
const COLORS = {{'baseline':'{colors["baseline"]}','exclusion':'{colors["exclusion"]}','correction':'{colors["correction"]}'}};
const LABELS = {{'baseline':'基准','exclusion':'NLOS排除','correction':'NLOS修正(ΔL)'}};
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
const nlosTs  = {nlos_ts_js};
const totalTs = {total_ts_js};

// CDF
new Chart(document.getElementById('cCDF'), {{
  type:'line',
  data:{{
    datasets: MODES.map(m => ({{
      label: LABELS[m],
      data: cdfData[m],
      borderColor: COLORS[m].replace('.7','.9').replace('.8','.9').replace('.85','1'),
      backgroundColor: 'transparent',
      borderWidth: 2.5,
      pointRadius: 0,
    }}))
  }},
  options:{{
    parsing:{{xAxisKey:'x',yAxisKey:'y'}},
    plugins:{{legend:{{position:'top'}}}},
    scales:{{
      x:{{title:{{display:true,text:'水平误差 (m)'}}}},
      y:{{title:{{display:true,text:'CDF'}},min:0,max:1}}
    }}
  }}
}});

// Bar
new Chart(document.getElementById('cBar'), {{
  type:'bar',
  data:{{
    labels: barLabels,
    datasets: MODES.map(m => ({{
      label: LABELS[m],
      data: barData[m],
      backgroundColor: COLORS[m]
    }}))
  }},
  options:{{
    plugins:{{legend:{{position:'top'}}}},
    scales:{{y:{{title:{{display:true,text:'误差 (m)'}}}}}}
  }}
}});

// Time series
new Chart(document.getElementById('cTS'), {{
  type:'line',
  data:{{
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
  options:{{
    parsing:{{xAxisKey:'x',yAxisKey:'y'}},
    plugins:{{legend:{{position:'top'}}}},
    scales:{{
      x:{{type:'linear',title:{{display:true,text:'UTC 时间戳 (s)'}}}},
      y:{{title:{{display:true,text:'水平误差 (m)'}},min:0}}
    }}
  }}
}});

// Satellite count
new Chart(document.getElementById('cSatCnt'), {{
  type:'line',
  data:{{
    datasets:[
      {{label:'总卫星数',data:totalTs,borderColor:'rgba(100,100,200,.8)',
        borderWidth:1.5,pointRadius:0,fill:false}},
      {{label:'NLOS 卫星数',data:nlosTs,borderColor:'rgba(244,67,54,.7)',
        borderWidth:1.5,pointRadius:0,fill:false,
        backgroundColor:'rgba(244,67,54,.1)',fill:true}}
    ]
  }},
  options:{{
    parsing:{{xAxisKey:'x',yAxisKey:'y'}},
    plugins:{{legend:{{position:'top'}}}},
    scales:{{
      x:{{type:'linear',title:{{display:true,text:'UTC 时间戳 (s)'}}}},
      y:{{title:{{display:true,text:'卫星数'}},min:0}}
    }}
  }}
}});
</script>
</body>
</html>"""

with open(args.out_html, 'w', encoding='utf-8') as f:
    f.write(html)
print(f'已保存 HTML：{args.out_html}')
