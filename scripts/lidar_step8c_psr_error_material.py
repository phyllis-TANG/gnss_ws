#!/usr/bin/env python3
"""
lidar_step8c_psr_error_material.py
转向：把预测目标从 CN0_drop 换成【伪距残差 psr_error】——米级强信号。

物理：NLOS 卫星追踪反射信号，测量伪距 = 真实距离 + ΔL（额外反射路径）
      → psr_error = psr_corr − r_gt − 接收机钟差 ≈ ΔL

验证三件事：
  1. psr_error vs ΔL（几何）：预期强正相关，且接近 y=x（验证 NLOS 反射模型）
  2. psr_error vs rho_norm（材质）：材质是否在几何之外有增量贡献
  3. 偏相关 psr_error vs rho_norm | ΔL：控制几何后材质还剩多少

钟差估计：每历元每星座用 LOS 卫星的 (psr_corr − r_gt) 中位数（复用 step8b）

输入:
  --obs, --nav_gps/bds/gal, --gt        (与 step8b 一致)
  --refl  lidar_reflection_intensity_mg.csv  (含 rho_norm, delta_L_m, severity)

输出:
  --out_csv   psr_error_material.csv
  --out_html  psr_error_material.html

用法:
  python3 lidar_step8c_psr_error_material.py \\
    --obs     /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \\
    --nav_gps /root/urbannav_gnss/hksc137c.21n \\
    --nav_bds /root/urbannav_gnss/hksc137c.21f \\
    --nav_gal /root/urbannav_gnss/hksc137c.21l \\
    --refl    /root/lidar_reflection_intensity_mg.csv \\
    --gt      /root/urbannav_gt.txt \\
    --out_csv  /root/psr_error_material.csv \\
    --out_html /root/psr_error_material.html
"""

import argparse, csv, json, math, os, sys
from collections import defaultdict
import numpy as np
from scipy import stats

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

LEAP_SECONDS = 18
C_LIGHT = 299792458.0
OMEGA_E = 7.2921151467e-5
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
ap.add_argument('--refl',     default='/root/lidar_reflection_intensity_mg.csv')
ap.add_argument('--gt',       default='/root/urbannav_gt.txt')
ap.add_argument('--out_csv',  default='/root/psr_error_material.csv')
ap.add_argument('--out_html', default='/root/psr_error_material.html')
ap.add_argument('--min_elev', type=float, default=10.0)
ap.add_argument('--gt_tol',   type=float, default=10.0)
ap.add_argument('--min_los',  type=int,   default=3)
args = ap.parse_args()

# ── helpers（复用 step8b）─────────────────────────────────────────────────────
def sagnac_correct(sat_ecef, tt):
    th = OMEGA_E * tt; c, s = math.cos(th), math.sin(th)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ sat_ecef

def tropo_delay(elev_deg):
    el = max(elev_deg, 3.0)
    return 2.3 / math.sin(math.radians(el) + 0.017)

def elev_from_ecef(rx, sat, lat, lon):
    diff = sat - rx; r = np.linalg.norm(diff)
    la, lo = math.radians(lat), math.radians(lon)
    up = np.array([math.cos(la)*math.cos(lo), math.cos(la)*math.sin(lo), math.sin(la)])
    return math.degrees(math.asin(np.clip(np.dot(diff/r, up), -1, 1)))

def load_nav(path, prefix):
    try:
        raw = read_rinex_nav(path)
    except Exception as e:
        print(f'  [WARN] {path}: {e}'); return {}
    out = {}
    for k, v in raw.items():
        if isinstance(k, int):            nk = f'{prefix}{k:02d}'
        elif isinstance(k, str) and k and k[0] != prefix: nk = prefix + k[1:]
        else:                             nk = k
        out[nk] = v
    return out

def dms_to_deg(d, m, s):
    return float(d) + float(m)/60 + float(s)/3600

TKEY = lambda t: round(float(t), 1)

# ── 读反射率（NLOS 源 + 材质）────────────────────────────────────────────────
print(f'读取反射率: {args.refl}')
refl = {}
with open(args.refl) as f:
    for r in csv.DictReader(f):
        if r.get('rho_norm', '') == '':
            continue
        try:
            rho = float(r['rho_norm'])
            if rho <= 0: continue
            info = {
                'rho_norm':  rho,
                'delta_L':   float(r['delta_L_m']) if r.get('delta_L_m') else np.nan,
                'incidence': float(r['incidence_deg']) if r.get('incidence_deg') else 45.0,
                'severity':  r.get('severity', ''),
                'sys':       r.get('sys', r['sat_id'][0]),
            }
            refl[(TKEY(r['unix_t']), r['sat_id'])] = info
            refl[(TKEY(r['utc_t']),  r['sat_id'])] = info
        except (ValueError, KeyError):
            continue
print(f'  {len(refl)//2} 条 NLOS 反射记录（含 rho_norm）')

# ── 读 GT ────────────────────────────────────────────────────────────────────
print(f'读取 GT: {args.gt}')
gt_t, gt_la, gt_lo, gt_al = [], [], [], []
with open(args.gt) as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('UTC') or line.startswith('('):
            continue
        p = line.split()
        if len(p) < 10: continue
        try:
            t = float(p[0])
            if t < 1e9: continue
            gt_t.append(t)
            gt_la.append(dms_to_deg(p[3], p[4], p[5]))
            gt_lo.append(dms_to_deg(p[6], p[7], p[8]))
            gt_al.append(float(p[9]))
        except (ValueError, IndexError):
            continue
gt_t = np.array(gt_t); gt_la = np.array(gt_la); gt_lo = np.array(gt_lo); gt_al = np.array(gt_al)
order = np.argsort(gt_t)
gt_t, gt_la, gt_lo, gt_al = gt_t[order], gt_la[order], gt_lo[order], gt_al[order]
print(f'  {len(gt_t)} GT 点')

def match_gt(utc_t):
    idx = int(np.searchsorted(gt_t, utc_t)); idx = min(max(idx, 0), len(gt_t)-1)
    if idx > 0 and abs(gt_t[idx-1]-utc_t) < abs(gt_t[idx]-utc_t): idx -= 1
    if abs(gt_t[idx]-utc_t) > args.gt_tol: return None
    return gt_la[idx], gt_lo[idx], gt_al[idx]

# ── 读 nav + obs ─────────────────────────────────────────────────────────────
print('读取星历...')
ephem = {}
ephem.update(load_nav(args.nav_gps, 'G'))
ephem.update(load_nav(args.nav_bds, 'C'))
ephem.update(load_nav(args.nav_gal, 'E'))
print(f'  {sum(len(v) for v in ephem.values())} 条星历 / {len(ephem)} 星')

print(f'读取 obs: {args.obs}')
obs_epochs = read_rinex_obs(args.obs)
print(f'  {len(obs_epochs)} 历元')

# ── 主循环 ───────────────────────────────────────────────────────────────────
out_rows = []
ep_skip = clk_fail = 0
for epoch in obs_epochs:
    rinex_t = epoch.time_unix
    utc_t   = rinex_t - LEAP_SECONDS
    gm = match_gt(utc_t)
    if gm is None:
        ep_skip += 1; continue
    gt_lat, gt_lon, gt_alt = gm
    gt_ecef = np.array(llh_to_ecef(gt_lat, gt_lon, gt_alt))

    sats = []
    for obs in epoch.obs_list:
        sat_id = obs.sat_id
        sysc = obs.sys if obs.sys else sat_id[0]
        if sysc not in ('G', 'C', 'E'): continue
        eph = find_closest_ephem(ephem.get(sat_id, []), rinex_t)
        if eph is None: continue
        sat_raw, dt_sv = compute_sat_position(eph, rinex_t)
        if sat_raw is None: continue
        psr = 0.0
        for k in PSR_KEYS.get(sysc, ['C1C']):
            if k in obs.pseudorange and obs.pseudorange[k] > 0:
                psr = obs.pseudorange[k]; break
        if psr <= 0: continue
        sat_s = sagnac_correct(np.array(sat_raw), psr / C_LIGHT)
        elev = elev_from_ecef(gt_ecef, sat_s, gt_lat, gt_lon)
        if elev < args.min_elev: continue
        psr_corr = psr + C_LIGHT * dt_sv - tropo_delay(elev)
        r_gt = float(np.linalg.norm(sat_s - gt_ecef))
        info = refl.get((TKEY(rinex_t), sat_id)) or refl.get((TKEY(utc_t), sat_id))
        sats.append({'sat_id': sat_id, 'sys': sysc, 'elev': elev,
                     'psr_corr': psr_corr, 'r_gt': r_gt,
                     'is_nlos': 1 if info else 0, 'info': info})

    if not sats: continue
    # 钟差：每星座 LOS 卫星 (psr_corr − r_gt) 中位数
    los = defaultdict(list)
    for s in sats:
        if s['is_nlos'] == 0:
            los[s['sys']].append(s['psr_corr'] - s['r_gt'])
    clk = {sy: float(np.median(v)) for sy, v in los.items() if len(v) >= args.min_los}
    if not clk:
        clk_fail += 1; continue

    for s in sats:
        if s['is_nlos'] == 1 and s['info'] and s['sys'] in clk:
            psr_err = s['psr_corr'] - s['r_gt'] - clk[s['sys']]
            out_rows.append({
                'utc_t': utc_t, 'sat_id': s['sat_id'], 'sys': s['sys'],
                'severity': s['info']['severity'], 'elev': s['elev'],
                'psr_error': psr_err,
                'delta_L': s['info']['delta_L'],
                'rho_norm': s['info']['rho_norm'],
                'incidence': s['info']['incidence'],
            })

print(f'\nNLOS 伪距残差样本: {len(out_rows)}  (跳过历元 {ep_skip}, 钟差失败 {clk_fail})')
if not out_rows:
    print('错误：无样本'); raise SystemExit(1)

psr  = np.array([r['psr_error'] for r in out_rows])
dL   = np.array([r['delta_L']   for r in out_rows])
rho  = np.array([r['rho_norm']  for r in out_rows])
sev  = np.array([r['severity']  for r in out_rows])
sysa = np.array([r['sys']       for r in out_rows])

valid = ~np.isnan(dL)
print(f'\npsr_error 统计: 中位={np.median(psr):.1f}m  '
      f'5th={np.percentile(psr,5):.1f}  95th={np.percentile(psr,95):.1f}m')
print(f'ΔL 统计:        中位={np.nanmedian(dL):.1f}m  '
      f'95th={np.nanpercentile(dL,95):.1f}m')

# ── 诊断 1: psr_error vs ΔL（几何验证）───────────────────────────────────────
print(f'\n── psr_error vs ΔL（几何）──')
sp_dL, p_dL = stats.spearmanr(psr[valid], dL[valid])
pe_dL, _    = stats.pearsonr(psr[valid], dL[valid])
print(f'  Spearman r = {sp_dL:+.3f}  (p={p_dL:.2e})')
print(f'  Pearson  r = {pe_dL:+.3f}')
# 线性拟合斜率（理想≈1：psr_error≈ΔL）
slope, intc = np.polyfit(dL[valid], psr[valid], 1)
print(f'  线性拟合: psr_error = {slope:.2f}·ΔL + {intc:.1f}  (理想斜率≈1)')

# ── 诊断 2: psr_error vs rho_norm（材质）─────────────────────────────────────
print(f'\n── psr_error vs rho_norm（材质）──')
sp_rho, p_rho = stats.spearmanr(psr, rho)
print(f'  Spearman r = {sp_rho:+.3f}  (p={p_rho:.2e})')

# ── 诊断 3: 偏相关 psr_error vs rho | ΔL ─────────────────────────────────────
def residualize(y, *cov):
    X = np.column_stack([np.ones(len(y))] + list(cov))
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    return y - X @ beta
log_rho = np.log(rho[valid])
pr = residualize(log_rho, dL[valid])
py = residualize(psr[valid], dL[valid])
pcorr, ppc = stats.pearsonr(pr, py)
print(f'\n── 偏相关 psr_error vs log(rho) | ΔL ──')
print(f'  r = {pcorr:+.3f}  (p={ppc:.3f})  '
      f'{"材质在几何之外有增量" if ppc<0.05 else "材质无显著增量贡献"}')

# ── 分星座/severity ──────────────────────────────────────────────────────────
print(f'\n── psr_error vs ΔL 分星座 ──')
for sy in ['G', 'C', 'E']:
    m = (sysa == sy) & valid
    if m.sum() < 20: continue
    r_s = stats.spearmanr(psr[m], dL[m])[0]
    print(f'  [{sy}] n={m.sum():4d}  r(psr,ΔL)={r_s:+.3f}  '
          f'psr中位={np.median(psr[m]):+.1f}m')

# ── 写 CSV ────────────────────────────────────────────────────────────────────
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['utc_t','sat_id','sys','severity','elev',
                                       'psr_error','delta_L','rho_norm','incidence'])
    w.writeheader()
    for r in out_rows:
        w.writerow({k: (f'{v:.3f}' if isinstance(v, float) else v) for k, v in r.items()})
print(f'\n写出 CSV: {args.out_csv}')

# ── 写 HTML（散点 psr_error vs ΔL）──────────────────────────────────────────
page = {
    'n': len(out_rows),
    'geom': {'sp': round(float(sp_dL),3), 'pe': round(float(pe_dL),3),
             'slope': round(float(slope),3), 'intc': round(float(intc),1)},
    'mat':  {'sp': round(float(sp_rho),3), 'pcorr': round(float(pcorr),3),
             'ppc': round(float(ppc),4)},
    'scatter': [{'dL': round(float(dL[i]),1), 'psr': round(float(psr[i]),1),
                 'sev': sev[i]}
                for i in np.where(valid)[0]],
}
html = """<!DOCTYPE html><html lang="zh"><head><meta charset="UTF-8">
<title>伪距误差 vs 材质/几何</title>
<style>
body{font-family:Arial;max-width:1000px;margin:20px auto;background:#f4f5f7;color:#333}
h1{color:#1565C0;border-bottom:3px solid #1565C0;padding-bottom:8px}
.card{background:#fff;border-radius:8px;padding:18px;box-shadow:0 1px 5px rgba(0,0,0,.1);margin-bottom:16px}
table{border-collapse:collapse;width:100%}th{background:#1565C0;color:#fff;padding:7px 10px;text-align:left}
td{padding:6px 10px;border-bottom:1px solid #eee}.big{font-size:1.3em;font-weight:bold;color:#1565C0}
canvas{display:block;margin:10px auto}.note{color:#666;font-size:.88em}
</style></head><body>
<h1>伪距残差 vs 材质/几何</h1>
<div class="card">
<p>NLOS 样本数: <span class="big" id="n"></span></p>
<table>
<tr><th>关系</th><th>Spearman</th><th>说明</th></tr>
<tr id="row_geom"></tr>
<tr id="row_mat"></tr>
<tr id="row_pc"></tr>
</table>
</div>
<div class="card">
<b>psr_error vs ΔL 散点（几何验证，红线=理想 y=x）</b>
<canvas id="sc" width="700" height="500"></canvas>
<p class="note">若点群沿 y=x 分布 → NLOS 反射模型成立，伪距误差就是额外路径 ΔL</p>
</div>
<script>
const D = __PAGE_DATA__;
document.getElementById('n').textContent = D.n;
document.getElementById('row_geom').innerHTML =
  '<td>psr_error vs ΔL（几何）</td><td class="big">'+D.geom.sp+'</td>'
  +'<td>拟合 psr_error = '+D.geom.slope+'·ΔL + '+D.geom.intc+'（理想斜率≈1）</td>';
document.getElementById('row_mat').innerHTML =
  '<td>psr_error vs rho_norm（材质）</td><td>'+D.mat.sp+'</td><td>原始材质相关</td>';
document.getElementById('row_pc').innerHTML =
  '<td>偏相关 vs log(rho) | ΔL</td><td>'+D.mat.pcorr+'</td>'
  +'<td>'+(D.mat.ppc<0.05?'材质在几何之外有增量贡献':'材质无显著增量')+'</td>';
(function(){
  const cv=document.getElementById('sc'),ctx=cv.getContext('2d');
  const W=cv.width,H=cv.height,p=50,data=D.scatter;
  const xs=data.map(d=>d.dL),ys=data.map(d=>d.psr);
  const xMax=Math.max.apply(null,xs)*1.05;
  const yLo=Math.min.apply(null,ys),yHi=Math.max.apply(null,ys);
  const lo=Math.min(0,yLo),hi=Math.max(xMax,yHi);
  const sx=v=>p+(v-0)/(xMax-0)*(W-2*p);
  const sy=v=>H-p-(v-lo)/(hi-lo)*(H-2*p);
  ctx.fillStyle='#fafafa';ctx.fillRect(0,0,W,H);
  // y=x 理想线
  ctx.strokeStyle='#c62828';ctx.lineWidth=2;ctx.setLineDash([6,4]);
  ctx.beginPath();ctx.moveTo(sx(0),sy(0));ctx.lineTo(sx(xMax),sy(xMax));ctx.stroke();
  ctx.setLineDash([]);
  const SC={mild:'#42A5F5',strong:'#FFA726',severe:'#EF5350'};
  data.forEach(d=>{ctx.beginPath();ctx.arc(sx(d.dL),sy(d.psr),2.5,0,2*Math.PI);
    ctx.fillStyle=(SC[d.sev]||'#888')+'aa';ctx.fill();});
  ctx.strokeStyle='#333';ctx.lineWidth=1.5;
  ctx.beginPath();ctx.moveTo(p,p);ctx.lineTo(p,H-p);ctx.lineTo(W-p,H-p);ctx.stroke();
  ctx.fillStyle='#333';ctx.font='12px Arial';ctx.textAlign='center';
  ctx.fillText('ΔL 预测额外路径 (m)',W/2,H-8);
  ctx.save();ctx.translate(14,H/2);ctx.rotate(-Math.PI/2);
  ctx.fillText('实测 psr_error (m)',0,0);ctx.restore();
  ['mild','strong','severe'].forEach((s,i)=>{const lx=p+10+i*90;
    ctx.beginPath();ctx.arc(lx,p+10,5,0,2*Math.PI);ctx.fillStyle=SC[s];ctx.fill();
    ctx.fillStyle='#555';ctx.textAlign='left';ctx.fillText(s,lx+9,p+14);});
})();
</script></body></html>"""
html = html.replace('__PAGE_DATA__', json.dumps(page))
with open(args.out_html, 'w') as f:
    f.write(html)
print(f'写出 HTML: {args.out_html}')

print(f'\n=== 核心结论 ===')
print(f'  几何: psr_error vs ΔL  Spearman = {sp_dL:+.3f}  (拟合斜率 {slope:.2f})')
print(f'  材质: psr_error vs rho  Spearman = {sp_rho:+.3f}')
print(f'  材质增量(控制ΔL后): r = {pcorr:+.3f}  p={ppc:.3f}')
if abs(sp_dL) > 0.4:
    print(f'  → ✅ 几何强信号确认：伪距误差由 ΔL 主导，比 CN0 强得多')
print(f'\n拷出: sudo docker cp ros1_gnss:{args.out_html} ~/psr_error_material.html')
