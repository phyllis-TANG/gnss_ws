#!/usr/bin/env python3
"""
lidar_step8_residual_analysis.py
用地面真値位置反算每颗卫星的真实伪距残差，与 Step 6 预测的 ΔL 逐条对比，
量化几何反射模型的准确性，为 SDR 实测验证提供基准框架。

核心思路：
  用已知的地面真値 (GT) 位置反算每颗卫星的“真实”距离 r_gt。
  伪距残差 = 校正后伪距 - r_gt - 时钟偏差
  对 LOS 卫星：残差接近 0（用于估计各星座时钟偏差）
  对 NLOS 卫星：残差 ≈ 实际多径延迟 → 与预测 ΔL 对比

输出：
  CSV: 每条 NLOS 卫星的 (predicted_dL, actual_residual, difference)
  HTML: 散点图、相关性分析、CDF、仰角分层统计

用法（容器内）：
  python3 lidar_step8_residual_analysis.py \\
    --obs   /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
    --nav_gps /root/urbannav_gnss/hksc137c.21n \\
    --nav_bds /root/urbannav_gnss/hksc137c.21f \\
    --nav_gal /root/urbannav_gnss/hksc137c.21l \\
    --nlos  /root/lidar_nlos_multignss.csv \\
    --refl  /root/lidar_reflection_multignss.csv \\
    --gt    /root/urbannav_gt.txt \\
    --out_csv  /root/residual_analysis.csv \\
    --out_html /root/residual_analysis.html
"""

import argparse, csv, math, os, sys, json
import numpy as np
from collections import defaultdict

# ── rinex_utils path ──────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in [
    os.path.join(SCRIPT_DIR, 'src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/scripts'),
    os.path.join(SCRIPT_DIR, 'PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts'),
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
OMEGA_E      = 7.2921151467e-5

PSR_KEYS = {
    'G': ['C1C', 'C1P', 'C1X', 'C2C', 'C2P'],
    'C': ['C1I', 'C1X', 'C1C', 'C2I', 'C7I'],
    'E': ['C1C', 'C1X', 'C1B', 'C5Q', 'C5X'],
}

ap = argparse.ArgumentParser(description='Step 8: LiDAR ΔL vs actual pseudorange residual')
ap.add_argument('--obs',     default='/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs')
ap.add_argument('--nav_gps', default='/root/urbannav_gnss/hksc137c.21n')
ap.add_argument('--nav_bds', default='/root/urbannav_gnss/hksc137c.21f')
ap.add_argument('--nav_gal', default='/root/urbannav_gnss/hksc137c.21l')
ap.add_argument('--nlos',    default='/root/lidar_nlos_multignss.csv')
ap.add_argument('--refl',    default='/root/lidar_reflection_multignss.csv')
ap.add_argument('--gt',      default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',  default='/root/residual_analysis.csv')
ap.add_argument('--out_html', default='/root/residual_analysis.html')
ap.add_argument('--min_elev', type=float, default=10.0)
ap.add_argument('--gt_tol',   type=float, default=10.0)
ap.add_argument('--min_los_sats', type=int, default=3,
                help='每星座最少 LOS 卫星数，不足则跳过该历元（默认3）')
args = ap.parse_args()


# ── helpers ────────────────────────────────────────────────────────────────
def sagnac_correct(sat_ecef, travel_time_s):
    theta = OMEGA_E * travel_time_s
    c, s  = math.cos(theta), math.sin(theta)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ sat_ecef

def tropo_delay(elev_deg):
    el = max(elev_deg, 3.0)
    return 2.3 / math.sin(math.radians(el) + 0.017)


def load_nav(path, prefix):
    try:
        raw = read_rinex_nav(path)
    except Exception as e:
        print(f'  [WARN] {path}: {e}')
        return {}
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

def elev_from_ecef(rx_ecef, sat_ecef, rx_lat, rx_lon):
    diff = sat_ecef - rx_ecef
    r    = np.linalg.norm(diff)
    lat, lon = math.radians(rx_lat), math.radians(rx_lon)
    up = np.array([math.cos(lat)*math.cos(lon),
                   math.cos(lat)*math.sin(lon),
                   math.sin(lat)])
    return math.degrees(math.asin(np.clip(np.dot(diff/r, up), -1, 1)))


# ── load NLOS / delta-L ─────────────────────────────────────────────────
print('加载 NLOS 标签...')
nlos_dict = {}
with open(args.nlos) as f:
    for row in csv.DictReader(f):
        nlos_dict[(row['unix_t'], row['sat_id'])] = int(row['lidar_nlos'])
print(f'  {len(nlos_dict)} 条')

print('加载反射面 ΔL...')
refl_dict = {}
with open(args.refl) as f:
    for row in csv.DictReader(f):
        dl   = float(row['delta_L_m']) if row['delta_L_m'] else 0.0
        plan = float(row['planarity']) if row['planarity'] else float('nan')
        refl_dict[(row['unix_t'], row['sat_id'])] = (dl, plan)
print(f'  {len(refl_dict)} 条')


# ── load ground truth ──────────────────────────────────────────────────────
print(f'加载地面真値: {args.gt}')
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
order    = np.argsort(gt_times)
gt_times, gt_lats, gt_lons, gt_alts = (
    gt_times[order], gt_lats[order], gt_lons[order], gt_alts[order])
print(f'  {len(gt_times)} 个 GT 点')

def match_gt(utc_t):
    idx = int(np.searchsorted(gt_times, utc_t))
    idx = min(max(idx, 0), len(gt_times)-1)
    if idx > 0 and abs(gt_times[idx-1]-utc_t) < abs(gt_times[idx]-utc_t):
        idx -= 1
    if abs(gt_times[idx]-utc_t) > args.gt_tol:
        return None
    return gt_lats[idx], gt_lons[idx], gt_alts[idx]


# ── load nav + obs ────────────────────────────────────────────────────────────────
print('读取导航电文...')
ephem = {}
ephem.update(load_nav(args.nav_gps, 'G'))
ephem.update(load_nav(args.nav_bds, 'C'))
ephem.update(load_nav(args.nav_gal, 'E'))
print(f'  {len(ephem)} 颗卫星')

print('读取观测文件...')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} 个历元')


# ── main loop ────────────────────────────────────────────────────────────────────
print(f'\n计算伪距残差（min_elev={args.min_elev}°）...')

out_rows   = []   # NLOS 卫星的分析结果
epoch_skip = 0
clk_fail   = 0

for ep_i, epoch in enumerate(obs_epochs):
    rinex_t = epoch.time_unix
    utc_t   = rinex_t - LEAP_SECONDS

    gt_match = match_gt(utc_t)
    if gt_match is None:
        epoch_skip += 1
        continue
    gt_lat, gt_lon, gt_alt = gt_match
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    # ── 处理每颗卫星 ─────────────────────────────────────────────────
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

        psr_corr = psr_raw + C_LIGHT * dt_sv - tropo_delay(elev)  # dt_sv 来自 compute_sat_position（正确）
        r_gt     = float(np.linalg.norm(sat_ecef_s - gt_ecef))

        key_nt  = (f'{rinex_t:.3f}', sat_id)
        is_nlos = nlos_dict.get(key_nt, 0)
        dl, plan = refl_dict.get(key_nt, (float('nan'), float('nan')))

        sats.append({
            'sat_id':   sat_id,
            'sys':      sys_char,
            'elev':     elev,
            'psr_corr': psr_corr,
            'r_gt':     r_gt,
            'is_nlos':  is_nlos,
            'delta_l':  dl,
            'planarity':plan,
            'utc_t':    utc_t,
            'rinex_t':  rinex_t,
        })

    if not sats:
        continue

    # ── 用 LOS 卫星估计每星座时钟偏差（中位数，对 NLOS 漏检鲁棒）────────────────
    # clk_bias[sys] = median(psr_corr - r_gt) over LOS sats of that sys
    # 中位数：当 "LOS" 集合中有少量 NLOS 漏检时，均值会被拉偏；中位数不受影响
    los_by_sys = defaultdict(list)
    for s in sats:
        if s['is_nlos'] == 0:
            los_by_sys[s['sys']].append(s['psr_corr'] - s['r_gt'])

    # 需要至少 3 颗 LOS 卫星，单颗时中位数无意义且容易被污染
    if not any(len(v) >= 3 for v in los_by_sys.values()):
        clk_fail += 1
        continue

    clk_bias = {sys: float(np.median(residuals))
                for sys, residuals in los_by_sys.items()
                if len(residuals) >= 3}

    # ── 计算每颗 NLOS 卫星的实际伪距误差 ────────────────────────────────────────
    for s in sats:
        if s['is_nlos'] != 1:
            continue
        sys = s['sys']
        if sys not in clk_bias:
            continue

        # 实际 NLOS 伪距误差（扣除时钟偏差后的残差）
        actual_error = s['psr_corr'] - s['r_gt'] - clk_bias[sys]
        predicted_dl = s['delta_l']
        difference   = actual_error - predicted_dl if not math.isnan(predicted_dl) else float('nan')

        out_rows.append({
            'utc_t':        f'{s["utc_t"]:.3f}',
            'sat_id':       s['sat_id'],
            'sys':          s['sys'],
            'elevation_deg':f'{s["elev"]:.2f}',
            'r_gt_m':       f'{s["r_gt"]:.3f}',
            'psr_corr_m':   f'{s["psr_corr"]:.3f}',
            'clk_bias_m':   f'{clk_bias[sys]:.3f}',
            'actual_error_m': f'{actual_error:.3f}',
            'predicted_dL_m': f'{predicted_dl:.3f}' if not math.isnan(predicted_dl) else '',
            'difference_m':   f'{difference:.3f}'   if not math.isnan(difference)  else '',
            'planarity':      f'{s["planarity"]:.3f}' if not math.isnan(s["planarity"]) else '',
        })

    if (ep_i + 1) % 100 == 0:
        print(f'\r  {ep_i+1}/{len(obs_epochs)} 历元，累积 {len(out_rows)} 条 NLOS 残差...', end='', flush=True)

print(f'\n完成')
print(f'  NLOS 残差记录  : {len(out_rows)}')
print(f'  跳过（无GT）   : {epoch_skip}')
print(f'  跳过（无LOS钟差）: {clk_fail}')


# ── 统计 ──────────────────────────────────────────────────────────────────────
valid = [r for r in out_rows if r['actual_error_m'] and r['predicted_dL_m']]
if not valid:
    print('无有效匹配记录，退出')
    sys.exit(0)

actual_errs = np.array([float(r['actual_error_m'])   for r in valid])
pred_dls    = np.array([float(r['predicted_dL_m'])   for r in valid])
diffs       = np.array([float(r['difference_m'])     for r in valid])
elevs       = np.array([float(r['elevation_deg'])    for r in valid])

# 相关系数
corr = float(np.corrcoef(pred_dls, actual_errs)[0, 1])

# 按仰角分筱
elev_bins = [(10, 20), (20, 30), (30, 45), (45, 60), (60, 90)]
bin_stats = []
for lo, hi in elev_bins:
    mask = (elevs >= lo) & (elevs < hi)
    sub_a = actual_errs[mask]
    sub_p = pred_dls[mask]
    if len(sub_a) < 3:
        bin_stats.append({'range': f'{lo}–{hi}°', 'n': len(sub_a),
                          'mean_actual': float('nan'), 'mean_pred': float('nan'),
                          'corr': float('nan'), 'bias': float('nan')})
        continue
    bc = float(np.corrcoef(sub_p, sub_a)[0, 1]) if len(sub_a) > 2 else float('nan')
    bin_stats.append({
        'range':       f'{lo}–{hi}°',
        'n':           int(len(sub_a)),
        'mean_actual': float(np.mean(sub_a)),
        'mean_pred':   float(np.mean(sub_p)),
        'corr':        bc,
        'bias':        float(np.mean(sub_a - sub_p)),  # predicted - actual 系统偏差
    })

print(f'\n--- 残差对比统计 ---')
print(f'  有效对数       : {len(valid)}')
print(f'  实际误差均値   : {np.mean(actual_errs):.2f}m  中位数 {np.median(actual_errs):.2f}m  std {np.std(actual_errs):.2f}m')
print(f'  预测 ΔL 均値   : {np.mean(pred_dls):.2f}m  中位数 {np.median(pred_dls):.2f}m  std {np.std(pred_dls):.2f}m')
print(f'  差値（actual-pred）均値: {np.mean(diffs):.2f}m  std {np.std(diffs):.2f}m')
print(f'  相关系数 r     : {corr:.3f}')
print(f'\n  --- 按仰角分层 ---')
for b in bin_stats:
    print(f'  {b["range"]:8s}  n={b["n"]:4d}  '
          f'actual={b["mean_actual"]:6.2f}m  '
          f'pred={b["mean_pred"]:6.2f}m  '
          f'bias={b["bias"]:+6.2f}m  '
          f'r={b["corr"]:.3f}')


# ── 写 CSV ────────────────────────────────────────────────────────────────────────
COLS = ['utc_t', 'sat_id', 'sys', 'elevation_deg', 'r_gt_m',
        'psr_corr_m', 'clk_bias_m', 'actual_error_m',
        'predicted_dL_m', 'difference_m', 'planarity']
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS)
    w.writeheader()
    w.writerows(out_rows)
print(f'\n已保存 CSV：{args.out_csv}')


# ── HTML 报告 ───────────────────────────────────────────────────────────────────
def make_scatter(xs, ys, labels=None, max_pts=3000):
    pts = []
    step = max(1, len(xs)//max_pts)
    for i in range(0, len(xs), step):
        d = {'x': round(float(xs[i]), 2), 'y': round(float(ys[i]), 2)}
        if labels:
            d['l'] = labels[i]
        pts.append(d)
    return json.dumps(pts)

def make_hist(vals, bins=30, clip_pct=99):
    v = [x for x in vals if not math.isnan(x)]
    lo, hi = float(np.percentile(v, 1)), float(np.percentile(v, clip_pct))
    step = (hi - lo) / bins
    counts = [0] * bins
    edges  = [lo + i*step for i in range(bins)]
    for x in v:
        i = min(int((x-lo)/step), bins-1)
        if 0 <= i < bins:
            counts[i] += 1
    labels = [f'{e:.1f}' for e in edges]
    return json.dumps(labels), json.dumps(counts)

sc_pred_actual   = make_scatter(pred_dls, actual_errs,
                                [r['sat_id'] for r in valid])
sc_elev_actual   = make_scatter(elevs, actual_errs,
                                [r['sat_id'] for r in valid])
sc_elev_pred     = make_scatter(elevs, pred_dls,
                                [r['sat_id'] for r in valid])
sc_pred_diff     = make_scatter(pred_dls, diffs,
                                [r['sat_id'] for r in valid])

hist_lbl_a, hist_cnt_a = make_hist(actual_errs.tolist())
hist_lbl_p, hist_cnt_p = make_hist(pred_dls.tolist())
hist_lbl_d, hist_cnt_d = make_hist(diffs.tolist())

bin_labels   = json.dumps([b['range']       for b in bin_stats])
bin_mean_a   = json.dumps([round(b['mean_actual'], 2) if not math.isnan(b['mean_actual']) else None for b in bin_stats])
bin_mean_p   = json.dumps([round(b['mean_pred'],   2) if not math.isnan(b['mean_pred'])   else None for b in bin_stats])
bin_bias     = json.dumps([round(b['bias'],        2) if not math.isnan(b['bias'])         else None for b in bin_stats])

html = f"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="UTF-8">
<title>Step 8 — LiDAR ΔL vs 实测伪距残差</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
body{{font-family:'Segoe UI',sans-serif;max-width:1100px;margin:0 auto;padding:20px;background:#f5f7fa;color:#333}}
h1{{color:#1a237e;border-bottom:3px solid #3f51b5;padding-bottom:8px}}
h2{{color:#283593;margin-top:24px}}h3{{color:#37474f;margin:0 0 5px}}
.section{{background:#fff;border-radius:10px;padding:20px;margin:14px 0;box-shadow:0 1px 5px rgba(0,0,0,.09)}}
.grid2{{display:grid;grid-template-columns:1fr 1fr;gap:20px}}
.grid3{{display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px}}
.kpi{{background:#e8eaf6;border-radius:8px;padding:16px;text-align:center}}
.kpi .v{{font-size:2em;font-weight:bold;color:#283593}}.kpi .l{{font-size:.82em;color:#666}}
table{{width:100%;border-collapse:collapse;font-size:.89em}}
th{{background:#283593;color:#fff;padding:9px 13px;text-align:left}}
td{{padding:7px 13px;border-bottom:1px solid #eee}}
tr:nth-child(even){{background:#f5f7ff}}
.formula{{background:#f8f9fa;border-left:4px solid #3f51b5;padding:9px 15px;font-family:monospace;margin:8px 0}}
.note{{font-size:.81em;color:#777;margin:3px 0 10px}}
.good{{color:#2e7d32;font-weight:bold}}.bad{{color:#c62828}}
</style></head><body>
<h1>Step 8 — LiDAR 几何 ΔL vs 实际伪距残差对比</h1>
<p>UrbanNav Medium-Urban-1 · NLOS 卫星分析 · 有效对数：<b>{len(valid)}</b></p>

<div class="section">
<h2>分析原理</h2>
<div class="formula">
实际伪距误差 = 校正伪距 − r_GT − 时钟偏差(星座)<br>
时钟偏差(星座) = mean(校正伪距 − r_GT) 对该星座 LOS 卫星取平均<br><br>
预测 ΔL = 2 × d_hit × cos(θ_i)  [Lau &amp; Cross 2007]<br>
差値 = 实际误差 − 预测 ΔL &nbsp;（正値=低估，负値=高估）
</div>
<p class="note">
注：r_GT 用地面真値位置（NovAtel SPAN CPT）计算，精度 &lt;0.05m。<br>
时钟偏差按星座独立估计：GPS、BeiDou、Galileo 各自用本星座 LOS 卫星均値。<br>
仅有地面真値匹配（容差 {args.gt_tol}s）且该星座有 ≥1 颗 LOS 卫星的历元参与分析。
</p>
</div>

<div class="section">
<h2>核心指标</h2>
<div class="grid3">
  <div class="kpi"><div class="v">{np.mean(actual_errs):.2f}m</div><div class="l">实际误差均値</div></div>
  <div class="kpi"><div class="v">{np.mean(pred_dls):.2f}m</div><div class="l">预测 ΔL 均値</div></div>
  <div class="kpi"><div class="v {'good' if abs(corr)>0.4 else 'bad'}">{corr:.3f}</div><div class="l">相关系数 r<br><small>|r|&gt;0.6 模型有效，|r|&lt;0.3 模型失效</small></div></div>
  <div class="kpi"><div class="v">{np.mean(diffs):+.2f}m</div><div class="l">系统偏差均値<br><small>正=低估，负=高估</small></div></div>
  <div class="kpi"><div class="v">{np.std(diffs):.2f}m</div><div class="l">差値标准差（随机误差）</div></div>
  <div class="kpi"><div class="v">{np.std(actual_errs):.2f}m</div><div class="l">实际误差标准差</div></div>
</div>
</div>

<div class="section">
<h2>按仰角分层统计</h2>
<p class="note">低仰角 NLOS 通常有更大延迟；模型在不同仰角的表现反映 PCA 反射面建模质量</p>
<table>
<tr><th>仰角段</th><th>样本数</th><th>实际误差均値</th><th>预测 ΔL 均値</th><th>系统偏差</th><th>相关系数 r</th></tr>
{''.join(
  f"<tr><td>{b['range']}</td><td>{b['n']}</td>"
  f"<td>{b['mean_actual']:.2f}m</td>"
  f"<td>{b['mean_pred']:.2f}m</td>"
  f"<td class='{'good' if abs(b['bias'])<3 else 'bad'}'>{b['bias']:+.2f}m</td>"
  f"<td class='{'good' if b['corr']>0.4 else 'bad'}'>{b['corr']:.3f}</td></tr>"
  if not math.isnan(b['mean_actual']) else
  f"<tr><td>{b['range']}</td><td>{b['n']}</td><td colspan='4'>样本不足</td></tr>"
  for b in bin_stats
)}
</table>
</div>

<div class="section">
<h2>散点图与分布</h2>
<div class="grid2">
  <div>
    <h3>预测 ΔL vs 实际误差（核心对比）</h3>
    <p class="note">理想情况：点沿对角线分布（预测=实际）<br>当前：相关系数 r = {corr:.3f}</p>
    <canvas id="cSc1" height="280"></canvas>
  </div>
  <div>
    <h3>仰角 vs 实际误差</h3>
    <p class="note">低仰角应有更大误差（NLOS 几何规律）</p>
    <canvas id="cSc2" height="280"></canvas>
  </div>
</div>
<div class="grid2" style="margin-top:16px">
  <div>
    <h3>仰角 vs 预测 ΔL</h3>
    <canvas id="cSc3" height="250"></canvas>
  </div>
  <div>
    <h3>预测 ΔL vs 差値（actual−pred）</h3>
    <p class="note">水平分布=随机误差；趋势线=系统性高估/低估</p>
    <canvas id="cSc4" height="250"></canvas>
  </div>
</div>
</div>

<div class="section">
<h2>分布直方图</h2>
<div class="grid3">
  <div><h3>实际伪距误差分布</h3><canvas id="cH1" height="200"></canvas></div>
  <div><h3>预测 ΔL 分布</h3><canvas id="cH2" height="200"></canvas></div>
  <div><h3>差値分布（actual−pred）</h3><canvas id="cH3" height="200"></canvas></div>
</div>
</div>

<div class="section">
<h2>按仰角分层：均値对比</h2>
<canvas id="cBin" height="120"></canvas>
</div>

<div class="section">
<h2>解读指南</h2>
<table>
<tr><th>相关系数 r</th><th>含义</th><th>对应行动</th></tr>
<tr><td class="good">&gt; 0.6</td><td>ΔL 几何模型有效，预测方向正确</td><td>可直接用于伪距改正</td></tr>
<tr><td>0.3 – 0.6</td><td>模型有部分参考价値</td><td>加入置信度权重后使用</td></tr>
<tr><td class="bad">&lt; 0.3</td><td>模型无效，预测随机</td><td>需要改进反射几何模型</td></tr>
</table>
<p class="note" style="margin-top:12px">
  系统偏差（actual−pred 均値）：正値表示 ΔL 低估了实际误差（信号比预测走了更多路程），
  通常由多次反射引起；负値表示高估，可能因为接收机实际跟踪到了直射路径。
</p>
<p class="note">
  <b>SDR 接入后</b>：将“实际误差”列替换为 SDR 相关器测量的延迟（Δτ × c），
  即可直接验证模型精度。本脚本的框架完全兼容。
</p>
</div>

<script>
const sc1={sc_pred_actual}, sc2={sc_elev_actual}, sc3={sc_elev_pred}, sc4={sc_pred_diff};
const h1l={hist_lbl_a}, h1c={hist_cnt_a};
const h2l={hist_lbl_p}, h2c={hist_cnt_p};
const h3l={hist_lbl_d}, h3c={hist_cnt_d};
const binL={bin_labels}, binA={bin_mean_a}, binP={bin_mean_p};

const scOpt = (xl,yl) => ({{
  plugins:{{legend:{{display:false}},
    tooltip:{{callbacks:{{label:d=>d.raw.l+` pred=${{d.raw.x}}m act=${{d.raw.y}}m`}}}}}},
  scales:{{x:{{title:{{display:true,text:xl}}}},y:{{title:{{display:true,text:yl}}}}}}
}});
const hOpt = lbl => ({{
  plugins:{{legend:{{display:false}}}},
  scales:{{x:{{title:{{display:true,text:lbl}},ticks:{{maxTicksLimit:7}}}},
           y:{{title:{{display:true,text:'频次'}}}}}}
}});

new Chart('cSc1',{{type:'scatter',
  data:{{datasets:[{{data:sc1,backgroundColor:'rgba(63,81,181,.35)',pointRadius:2.5}}]}},
  options:scOpt('预测 ΔL (m)','实际误差 (m)')}});
new Chart('cSc2',{{type:'scatter',
  data:{{datasets:[{{data:sc2,backgroundColor:'rgba(244,67,54,.35)',pointRadius:2.5}}]}},
  options:scOpt('仰角 (°)','实际误差 (m)')}});
new Chart('cSc3',{{type:'scatter',
  data:{{datasets:[{{data:sc3,backgroundColor:'rgba(0,150,136,.35)',pointRadius:2.5}}]}},
  options:scOpt('仰角 (°)','预测 ΔL (m)')}});
new Chart('cSc4',{{type:'scatter',
  data:{{datasets:[{{data:sc4,backgroundColor:'rgba(255,152,0,.4)',pointRadius:2.5}}]}},
  options:scOpt('预测 ΔL (m)','差値 actual−pred (m)')}});

new Chart('cH1',{{type:'bar',data:{{labels:h1l,datasets:[{{data:h1c,backgroundColor:'rgba(244,67,54,.65)'}}]}},options:hOpt('实际误差 (m)')}});
new Chart('cH2',{{type:'bar',data:{{labels:h2l,datasets:[{{data:h2c,backgroundColor:'rgba(63,81,181,.65)'}}]}},options:hOpt('预测 ΔL (m)')}});
new Chart('cH3',{{type:'bar',data:{{labels:h3l,datasets:[{{data:h3c,backgroundColor:'rgba(76,175,80,.65)'}}]}},options:hOpt('差値 actual−pred (m)')}});

new Chart('cBin',{{type:'bar',
  data:{{labels:binL,datasets:[
    {{label:'实际误差均値',data:binA,backgroundColor:'rgba(244,67,54,.7)'}},
    {{label:'预测 ΔL 均値',data:binP,backgroundColor:'rgba(63,81,181,.7)'}}
  ]}},
  options:{{plugins:{{legend:{{position:'top'}}}},
    scales:{{y:{{title:{{display:true,text:'误差 (m)'}}}}}}
  }}
}});
</script>
</body></html>"""

with open(args.out_html, 'w', encoding='utf-8') as f:
    f.write(html)
print(f'已保存 HTML：{args.out_html}')
print('\n拷出容器：')
print(f'  sudo docker cp ros1_gnss:/root/residual_analysis.html ~/residual_analysis.html')
