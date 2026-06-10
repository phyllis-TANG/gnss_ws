#!/usr/bin/env python3
"""
lidar_step6c_rho_vs_cn0.py
分析 LiDAR 材质反射率 ρ_norm 与 GNSS CN0 衰减的相关性。

核心假设：
  高 ρ_norm（玻璃/光滑墙面）→ NLOS 信号衰减少 → CN0_drop 小
  低 ρ_norm（混凝土/粗糙面）→ NLOS 信号衰减多 → CN0_drop 大

CN0_drop 定义：
  CN0_expected(elevation) 用多项式拟合 LOS 卫星的 CN0-elevation 关系
  CN0_drop = CN0_expected - CN0_measured（正值=衰减，负值=异常增强）

输入:
  --refl   lidar_reflection_intensity.csv  (step6b 输出，含 rho_norm)
  --gnss   training_data.csv               (del2AINLOS 输出，含 cn0, elevation, nlos_label)

输出:
  --out_csv   rho_cn0_analysis.csv
  --out_html  rho_cn0_analysis.html

用法:
  python3 lidar_step6c_rho_vs_cn0.py \
    --refl /root/lidar_reflection_intensity.csv \
    --gnss /root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/data/UrbanNavMedium/training_data.csv \
    --out_csv  /root/rho_cn0_analysis.csv \
    --out_html /root/rho_cn0_analysis.html
"""

import argparse, csv, math, os
import numpy as np
from scipy import stats

ap = argparse.ArgumentParser()
ap.add_argument('--refl',     default='/root/lidar_reflection_intensity.csv')
ap.add_argument('--gnss',     default='/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/data/UrbanNavMedium/training_data.csv')
ap.add_argument('--out_csv',  default='/root/rho_cn0_analysis.csv')
ap.add_argument('--out_html', default='/root/rho_cn0_analysis.html')
ap.add_argument('--cn0_poly', type=int, default=2,
                help='CN0-elevation 拟合多项式次数（默认 2）')
args = ap.parse_args()

# ── 读 GNSS 数据 ──────────────────────────────────────────────────────────────
print('读取 GNSS 数据...')
gnss_rows = []
with open(args.gnss) as f:
    for r in csv.DictReader(f):
        try:
            gnss_rows.append({
                'epoch':      float(r['epoch']),
                'sat_id':     r['sat_id'],
                'cn0':        float(r['cn0']),
                'elevation':  float(r['elevation']),
                'nlos_label': int(r['nlos_label'])
            })
        except (ValueError, KeyError):
            pass
print(f'  {len(gnss_rows)} 条，LOS: {sum(r["nlos_label"]==0 for r in gnss_rows)}, '
      f'NLOS: {sum(r["nlos_label"]==1 for r in gnss_rows)}')

# ── 用 LOS 卫星拟合 CN0-elevation 基准曲线 ────────────────────────────────────
los = [r for r in gnss_rows if r['nlos_label'] == 0 and r['cn0'] > 0]
elev_los = np.array([r['elevation'] for r in los])
cn0_los  = np.array([r['cn0']       for r in los])

poly_coeffs = np.polyfit(elev_los, cn0_los, args.cn0_poly)
cn0_model   = np.poly1d(poly_coeffs)
print(f'CN0-elevation 拟合（LOS {len(los)} 点，{args.cn0_poly}次多项式）:')
print(f'  在 10°: {cn0_model(10):.1f} dB-Hz')
print(f'  在 30°: {cn0_model(30):.1f} dB-Hz')
print(f'  在 60°: {cn0_model(60):.1f} dB-Hz')

# 为每条 GNSS 记录计算 CN0_drop
for r in gnss_rows:
    r['cn0_expected'] = float(cn0_model(r['elevation']))
    r['cn0_drop']     = r['cn0_expected'] - r['cn0']   # 正=衰减

# 建 GNSS 查询字典：(epoch_round, sat_id) → cn0_drop
gnss_dict = {}
for r in gnss_rows:
    key = (round(r['epoch'], 2), r['sat_id'])
    gnss_dict[key] = r

# ── 读 reflection intensity ──────────────────────────────────────────────────
print('读取反射率数据...')
refl_rows = []
with open(args.refl) as f:
    for r in csv.DictReader(f):
        if r.get('rho_norm', '') == '':
            continue
        try:
            refl_rows.append({
                'epoch':     round(float(r['utc_t']), 2),
                'sat_id':    r['sat_id'],
                'rho_norm':  float(r['rho_norm']),
                'intensity': float(r['intensity']) if r.get('intensity') else np.nan,
                'hit_dist':  float(r['hit_dist_m']),
                'incidence': float(r['incidence_deg']) if r.get('incidence_deg') else 45.0,
                'delta_L':   float(r['delta_L_m']) if r.get('delta_L_m') else np.nan,
                'severity':  r.get('severity', ''),
                'planarity': float(r['planarity']) if r.get('planarity') else np.nan,
            })
        except (ValueError, KeyError):
            pass
print(f'  {len(refl_rows)} 条有效反射率记录')

# ── 连接：reflection → GNSS ──────────────────────────────────────────────────
merged = []
for r in refl_rows:
    key = (r['epoch'], r['sat_id'])
    if key in gnss_dict:
        g = gnss_dict[key]
        merged.append({**r,
                       'cn0':          g['cn0'],
                       'cn0_expected': g['cn0_expected'],
                       'cn0_drop':     g['cn0_drop'],
                       'elevation':    g['elevation'],
                       'nlos_label':   g['nlos_label']})

print(f'匹配到 GNSS: {len(merged)} / {len(refl_rows)} ({100*len(merged)/len(refl_rows):.1f}%)')

if len(merged) == 0:
    print('错误：无匹配，请检查 epoch 对齐（utc_t vs epoch 字段）')
    exit(1)

rho   = np.array([r['rho_norm']  for r in merged])
drop  = np.array([r['cn0_drop']  for r in merged])
cn0   = np.array([r['cn0']       for r in merged])
elev  = np.array([r['elevation'] for r in merged])
delta = np.array([r['delta_L']   for r in merged])
sev   = np.array([r['severity']  for r in merged])

# ── 相关性分析 ────────────────────────────────────────────────────────────────
print('\n=== ρ_norm vs CN0_drop 相关性 ===')

# 全样本
r_pearson, p_pearson   = stats.pearsonr(rho, drop)
r_spearman, p_spearman = stats.spearmanr(rho, drop)
print(f'全样本 (n={len(merged)}):')
print(f'  Pearson  r = {r_pearson:.3f}  p = {p_pearson:.2e}')
print(f'  Spearman r = {r_spearman:.3f}  p = {p_spearman:.2e}')

# 按 severity 分组
print('\n按 severity 分组:')
for s in ['mild', 'strong', 'severe']:
    mask = sev == s
    if mask.sum() < 10:
        continue
    r_s, p_s = stats.spearmanr(rho[mask], drop[mask])
    print(f'  {s:7s} n={mask.sum():4d}  Spearman r={r_s:+.3f}  p={p_s:.2e}  '
          f'ρ中位={np.median(rho[mask]):.2f}  CN0_drop均值={np.mean(drop[mask]):.1f}dB')

# ρ_norm 分位数 → CN0_drop 均值（核心关系表）
print('\nρ_norm 分位数 vs CN0_drop:')
pcts = [0, 20, 40, 60, 80, 100]
for i in range(len(pcts)-1):
    lo, hi = np.percentile(rho, pcts[i]), np.percentile(rho, pcts[i+1])
    mask = (rho >= lo) & (rho < hi)
    if mask.sum() == 0: continue
    print(f'  ρ [{lo:.2f}, {hi:.2f})  n={mask.sum():4d}  '
          f'CN0_drop均值={np.mean(drop[mask]):+.2f}dB  CN0均值={np.mean(cn0[mask]):.1f}dB')

# ── 写出分析 CSV ──────────────────────────────────────────────────────────────
print(f'\n写出 CSV: {args.out_csv}')
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=[
        'epoch','sat_id','severity','rho_norm','intensity',
        'hit_dist_m','incidence_deg','delta_L_m',
        'cn0','cn0_expected','cn0_drop','elevation','nlos_label'])
    w.writeheader()
    for r in merged:
        w.writerow({
            'epoch':        f"{r['epoch']:.3f}",
            'sat_id':       r['sat_id'],
            'severity':     r['severity'],
            'rho_norm':     f"{r['rho_norm']:.4f}",
            'intensity':    f"{r['intensity']:.1f}" if not math.isnan(r['intensity']) else '',
            'hit_dist_m':   f"{r['hit_dist']:.2f}",
            'incidence_deg':f"{r['incidence']:.1f}",
            'delta_L_m':    f"{r['delta_L']:.3f}" if not math.isnan(r['delta_L']) else '',
            'cn0':          f"{r['cn0']:.1f}",
            'cn0_expected': f"{r['cn0_expected']:.1f}",
            'cn0_drop':     f"{r['cn0_drop']:.2f}",
            'elevation':    f"{r['elevation']:.1f}",
            'nlos_label':   r['nlos_label'],
        })

# ── HTML 报告 ─────────────────────────────────────────────────────────────────
print(f'写出 HTML: {args.out_html}')

# 散点图数据（最多 2000 点）
idx = np.random.choice(len(merged), min(2000, len(merged)), replace=False)
sc_rho  = rho[idx].tolist()
sc_drop = drop[idx].tolist()
sc_sev  = sev[idx].tolist()
sc_cn0  = cn0[idx].tolist()

color_map = {'mild': '#2196F3', 'strong': '#FF9800', 'severe': '#F44336', '': '#888888'}

html = f"""<!DOCTYPE html><html><head>
<meta charset="utf-8">
<title>ρ_norm vs CN0_drop — LiDAR Reflectance × GNSS Signal</title>
<style>
body{{font-family:Arial,sans-serif;margin:20px;background:#f5f5f5}}
.card{{background:white;border-radius:8px;padding:20px;margin:16px 0;box-shadow:0 2px 4px rgba(0,0,0,.1)}}
h1{{color:#1565C0}} h2{{color:#1976D2;margin-top:0}}
table{{border-collapse:collapse;width:100%}} th,td{{border:1px solid #ddd;padding:8px;text-align:right}}
th{{background:#1976D2;color:white}} tr:nth-child(even){{background:#f9f9f9}}
</style>
</head><body>
<h1>LiDAR 材质反射率 ρ_norm vs GNSS CN0 衰减</h1>
<p>数据集: UrbanNav HK Medium-Urban-1 | 匹配样本: {len(merged)} 条</p>

<div class="card">
<h2>相关性统计</h2>
<table><tr><th>指标</th><th>值</th><th>p值</th><th>解释</th></tr>
<tr><td>Pearson r (全样本)</td><td>{r_pearson:.3f}</td><td>{p_pearson:.2e}</td>
    <td>线性相关</td></tr>
<tr><td>Spearman ρ (全样本)</td><td>{r_spearman:.3f}</td><td>{p_spearman:.2e}</td>
    <td>单调相关</td></tr>
</table>
</div>

<div class="card">
<h2>散点图: ρ_norm vs CN0_drop</h2>
<canvas id="scatter" width="800" height="450"></canvas>
<p style="font-size:12px;color:#666">
蓝=mild(ΔL&lt;2m) | 橙=strong(2-10m) | 红=severe(&gt;10m) | 最多显示2000点
</p>
</div>

<div class="card">
<h2>ρ_norm 分位数 → CN0_drop 均值</h2>
<canvas id="bar" width="700" height="350"></canvas>
</div>

<script>
// ── 散点图 ──────────────────────────────────────────────────────────────────
const sc = document.getElementById('scatter');
const ctx = sc.getContext('2d');
const rho_data  = {sc_rho};
const drop_data = {sc_drop};
const sev_data  = {sc_sev};
const cmap = {{mild:'#2196F3', strong:'#FF9800', severe:'#F44336', '':'#888'}};

const rhoMin = Math.min(...rho_data), rhoMax = Math.min(Math.max(...rho_data), 15);
const dropMin = Math.min(...drop_data), dropMax = Math.max(...drop_data);
const pad = 60;
const W = sc.width - 2*pad, H = sc.height - 2*pad;

function sx(v){{ return pad + (Math.min(v,15)-rhoMin)/(rhoMax-rhoMin+0.001)*W; }}
function sy(v){{ return pad + H - (v-dropMin)/(dropMax-dropMin+0.001)*H; }}

ctx.fillStyle='#f8f8f8'; ctx.fillRect(0,0,sc.width,sc.height);
// grid
ctx.strokeStyle='#ddd'; ctx.lineWidth=0.5;
for(let v=-20;v<=20;v+=5){{
    ctx.beginPath(); ctx.moveTo(pad,sy(v)); ctx.lineTo(pad+W,sy(v)); ctx.stroke();
}}
// axes
ctx.strokeStyle='#333'; ctx.lineWidth=1.5;
ctx.beginPath(); ctx.moveTo(pad,pad); ctx.lineTo(pad,pad+H); ctx.lineTo(pad+W,pad+H); ctx.stroke();
// zero line
ctx.strokeStyle='#999'; ctx.setLineDash([4,4]);
ctx.beginPath(); ctx.moveTo(pad,sy(0)); ctx.lineTo(pad+W,sy(0)); ctx.stroke();
ctx.setLineDash([]);
// points
for(let i=0;i<rho_data.length;i++){{
    ctx.fillStyle = cmap[sev_data[i]]||'#888';
    ctx.globalAlpha = 0.5;
    ctx.beginPath();
    ctx.arc(sx(rho_data[i]), sy(drop_data[i]), 3, 0, 2*Math.PI);
    ctx.fill();
}}
ctx.globalAlpha=1;
// labels
ctx.fillStyle='#333'; ctx.font='13px Arial'; ctx.textAlign='center';
ctx.fillText('ρ_norm（材质反射率，相对值）', pad+W/2, sc.height-8);
ctx.save(); ctx.translate(14, pad+H/2); ctx.rotate(-Math.PI/2);
ctx.fillText('CN0_drop (dB-Hz)', 0, 0); ctx.restore();

// ── 分位数柱状图 ─────────────────────────────────────────────────────────────
const bar = document.getElementById('bar');
const bc  = bar.getContext('2d');
const pcts = [0,20,40,60,80,100];
const rhoSorted = [...rho_data].sort((a,b)=>a-b);
function pctVal(p){{ return rhoSorted[Math.floor(p/100*(rhoSorted.length-1))]; }}

const bins = [];
for(let i=0;i<5;i++){{
    const lo=pctVal(pcts[i]), hi=pctVal(pcts[i+1]);
    const inBin = rho_data.map((r,j)=>r>=lo&&r<hi?drop_data[j]:null).filter(v=>v!==null);
    const mean = inBin.reduce((a,b)=>a+b,0)/inBin.length;
    bins.push({{lo:lo.toFixed(1), hi:hi.toFixed(1), mean:mean, n:inBin.length}});
}}

const bW=bar.width, bH=bar.height, bPad=55;
const bW2=bW-2*bPad, bH2=bH-2*bPad;
const means = bins.map(b=>b.mean);
const mMin=Math.min(...means)-2, mMax=Math.max(...means)+2;
const barW = bW2/bins.length*0.6, barGap = bW2/bins.length;

bc.fillStyle='#f8f8f8'; bc.fillRect(0,0,bW,bH);
bc.strokeStyle='#ddd'; bc.lineWidth=0.5;
for(let v=Math.ceil(mMin);v<=Math.floor(mMax);v+=2){{
    const yy = bPad + bH2 - (v-mMin)/(mMax-mMin)*bH2;
    bc.beginPath(); bc.moveTo(bPad,yy); bc.lineTo(bPad+bW2,yy); bc.stroke();
}}
// zero line
bc.strokeStyle='#999'; bc.setLineDash([4,4]);
const y0 = bPad+bH2-(0-mMin)/(mMax-mMin)*bH2;
bc.beginPath(); bc.moveTo(bPad,y0); bc.lineTo(bPad+bW2,y0); bc.stroke();
bc.setLineDash([]);

const colors = ['#1E88E5','#42A5F5','#90CAF9','#FFB74D','#EF5350'];
bins.forEach((b,i)=>{{
    const x = bPad + i*barGap + barGap*0.2;
    const yTop = bPad + bH2 - (b.mean-mMin)/(mMax-mMin)*bH2;
    const h = Math.abs(yTop - y0);
    bc.fillStyle = colors[i];
    bc.fillRect(x, Math.min(yTop,y0), barW, h);
    bc.fillStyle='#333'; bc.font='11px Arial'; bc.textAlign='center';
    bc.fillText(`[${{b.lo}},${{b.hi}})`, x+barW/2, bH-10);
    bc.fillText(`n=${{b.n}}`, x+barW/2, bH-25);
    bc.fillStyle='white'; bc.font='bold 12px Arial';
    bc.fillText(b.mean.toFixed(1), x+barW/2, Math.min(yTop,y0)+15);
}});
bc.fillStyle='#333'; bc.font='13px Arial'; bc.textAlign='center';
bc.fillText('ρ_norm 分位区间（低→高反射率）', bPad+bW2/2, bH-2);
bc.save(); bc.translate(14, bPad+bH2/2); bc.rotate(-Math.PI/2);
bc.fillText('CN0_drop 均值 (dB-Hz)', 0, 0); bc.restore();
</script>
</body></html>"""

with open(args.out_html, 'w') as f:
    f.write(html)

print('\n=== 完成 ===')
print(f'CSV:  {args.out_csv}')
print(f'HTML: {args.out_html}')
print(f'\n核心发现摘要:')
print(f'  Pearson r  = {r_pearson:.3f} (ρ_norm vs CN0_drop)')
print(f'  Spearman ρ = {r_spearman:.3f}')
if abs(r_spearman) > 0.3:
    print('  → 显著相关：LiDAR反射率对CN0衰减有预测能力 ✓')
elif abs(r_spearman) > 0.1:
    print('  → 弱相关：有信号但噪声较大，需要更多数据或特征工程')
else:
    print('  → 相关性不显著：可能需要更复杂的模型或其他特征')
