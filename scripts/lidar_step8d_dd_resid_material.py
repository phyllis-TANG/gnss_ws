#!/usr/bin/env python3
"""
lidar_step8d_dd_resid_material.py
用【双差残差 dd_resid】作为干净的 NLOS 伪距偏差，重做几何/材质关联。

为什么用 DD：step8c 用 LOS 中位数估钟差，城市峡谷里缺干净参考 → 钟差污染
（出现物理不可能的负误差）。DD 用 HKSC 参考站站间双差，从根上消掉接收机/
卫星钟差，dd_resid ≈ NLOS 卫星的额外路径 ΔL，是干净的米级信号。

输入:
  --dd    dd_nlos_labels.csv          (step6b_dd_nlos 输出，含 dd_resid)
  --refl  lidar_reflection_intensity_mg.csv  (含 rho_norm, delta_L_m, severity)

分析:
  1. dd_resid vs ΔL（几何）：预期正相关，斜率≈1 → 验证 NLOS 反射模型
  2. dd_resid vs rho_norm（材质）
  3. 偏相关 dd_resid vs log(rho) | ΔL
  GPS-only 重点看（DD 在 GPS 上最干净）

用法:
  python3 lidar_step8d_dd_resid_material.py \\
    --dd /root/dd_nlos_labels.csv \\
    --refl /root/lidar_reflection_intensity_mg.csv \\
    --out_csv /root/dd_resid_material.csv \\
    --out_html /root/dd_resid_material.html
"""

import argparse, csv, json, math
import numpy as np
from scipy import stats

ap = argparse.ArgumentParser()
ap.add_argument('--dd',       default='/root/dd_nlos_labels.csv')
ap.add_argument('--refl',     default='/root/lidar_reflection_intensity_mg.csv')
ap.add_argument('--out_csv',  default='/root/dd_resid_material.csv')
ap.add_argument('--out_html', default='/root/dd_resid_material.html')
args = ap.parse_args()

TKEY = lambda t: round(float(t), 1)

# ── 读 DD 残差 ────────────────────────────────────────────────────────────────
print(f'读取 DD: {args.dd}')
dd = {}
with open(args.dd) as f:
    for r in csv.DictReader(f):
        try:
            dd[(TKEY(r['utc_t']), r['sat_id'])] = {
                'dd_resid': float(r['dd_resid']),
                'sys':      r.get('sys', r['sat_id'][0]),
                'elev':     float(r['elev_deg']) if r.get('elev_deg') else np.nan,
            }
        except (ValueError, KeyError):
            continue
print(f'  {len(dd)} 条 DD 记录')

# ── 读反射率（材质 + ΔL）─────────────────────────────────────────────────────
print(f'读取反射率: {args.refl}')
merged = []
with open(args.refl) as f:
    for r in csv.DictReader(f):
        if r.get('rho_norm', '') == '':
            continue
        try:
            rho = float(r['rho_norm'])
            if rho <= 0: continue
            key = (TKEY(r['utc_t']), r['sat_id'])
            d = dd.get(key)
            if d is None:
                continue
            merged.append({
                'utc_t':    float(r['utc_t']),
                'sat_id':   r['sat_id'],
                'sys':      d['sys'],
                'severity': r.get('severity', ''),
                'dd_resid': d['dd_resid'],
                'delta_L':  float(r['delta_L_m']) if r.get('delta_L_m') else np.nan,
                'rho_norm': rho,
                'incidence':float(r['incidence_deg']) if r.get('incidence_deg') else 45.0,
                'elev':     d['elev'],
            })
        except (ValueError, KeyError):
            continue
print(f'  匹配成功: {len(merged)} 条（反射点 ∩ DD）')
if not merged:
    print('错误：无匹配。请先运行 step6b_dd_nlos.py 生成 dd_nlos_labels.csv')
    raise SystemExit(1)

dr   = np.array([m['dd_resid'] for m in merged])
dL   = np.array([m['delta_L']  for m in merged])
rho  = np.array([m['rho_norm'] for m in merged])
sev  = np.array([m['severity'] for m in merged])
sysa = np.array([m['sys']      for m in merged])
valid = ~np.isnan(dL)

print(f'\ndd_resid 统计: 中位={np.median(dr):.1f}m  '
      f'5th={np.percentile(dr,5):.1f}  95th={np.percentile(dr,95):.1f}m')
print(f'ΔL 统计:       中位={np.nanmedian(dL):.1f}m  95th={np.nanpercentile(dL,95):.1f}m')

def analyze(label, mask):
    if mask.sum() < 20:
        print(f'  [{label}] n={mask.sum()} 样本不足'); return None
    d, l, rh = dr[mask], dL[mask], rho[mask]
    v = ~np.isnan(l)
    if v.sum() < 20:
        print(f'  [{label}] 有效ΔL不足'); return None
    sp = stats.spearmanr(d[v], l[v])[0]
    pe = stats.pearsonr(d[v], l[v])[0]
    slope, intc = np.polyfit(l[v], d[v], 1)
    sp_rho = stats.spearmanr(d, rh)[0]
    print(f'  [{label:14s}] n={mask.sum():4d}  '
          f'dd~ΔL: Sp={sp:+.3f} Pe={pe:+.3f} 斜率={slope:.2f}  | '
          f'dd~ρ: Sp={sp_rho:+.3f}')
    return {'label': label, 'n': int(mask.sum()),
            'sp_dL': round(float(sp),3), 'pe_dL': round(float(pe),3),
            'slope': round(float(slope),3), 'sp_rho': round(float(sp_rho),3)}

# ── 几何验证：全样本 + 分星座 + 分severity ───────────────────────────────────
print(f'\n── dd_resid vs ΔL（几何验证）──')
results = []
results.append(analyze('全样本', np.ones(len(merged), bool)))
for sy in ['G', 'C', 'E']:
    results.append(analyze(f'星座 {sy}', sysa == sy))
for sv in ['mild', 'strong', 'severe']:
    results.append(analyze(f'{sv}', sev == sv))
results = [r for r in results if r]

# ── 偏相关：GPS-only，控制 ΔL 后材质增量 ─────────────────────────────────────
def residualize(y, *cov):
    X = np.column_stack([np.ones(len(y))] + list(cov))
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    return y - X @ beta

g = (sysa == 'G') & valid
pcorr_g = ppc_g = float('nan')
if g.sum() >= 20:
    lr = np.log(rho[g]); pr = residualize(lr, dL[g]); py = residualize(dr[g], dL[g])
    pcorr_g, ppc_g = stats.pearsonr(pr, py)
    print(f'\n── GPS-only 偏相关 dd_resid vs log(ρ) | ΔL ──')
    print(f'  r={pcorr_g:+.3f}  p={ppc_g:.3f}  '
          f'{"材质有增量贡献" if ppc_g<0.05 else "材质无显著增量"}')

# ── 写 CSV ────────────────────────────────────────────────────────────────────
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['utc_t','sat_id','sys','severity',
                                       'dd_resid','delta_L','rho_norm','incidence','elev'])
    w.writeheader()
    for m in merged:
        w.writerow({k: (f'{v:.3f}' if isinstance(v, float) else v) for k, v in m.items()})
print(f'\n写出 CSV: {args.out_csv}')

# ── HTML 散点 dd_resid vs ΔL ─────────────────────────────────────────────────
page = {
    'n': len(merged),
    'results': results,
    'gps_partial': {'r': round(float(pcorr_g),3) if not math.isnan(pcorr_g) else None,
                    'p': round(float(ppc_g),4) if not math.isnan(ppc_g) else None},
    'scatter': [{'dL': round(float(dL[i]),1), 'dd': round(float(dr[i]),1),
                 'sev': sev[i], 'sys': sysa[i]}
                for i in np.where(valid)[0]],
}
html = """<!DOCTYPE html><html lang="zh"><head><meta charset="UTF-8">
<title>DD残差 vs 材质/几何</title>
<style>
body{font-family:Arial;max-width:1000px;margin:20px auto;background:#f4f5f7;color:#333}
h1{color:#1565C0;border-bottom:3px solid #1565C0;padding-bottom:8px}
.card{background:#fff;border-radius:8px;padding:18px;box-shadow:0 1px 5px rgba(0,0,0,.1);margin-bottom:16px}
table{border-collapse:collapse;width:100%}th{background:#1565C0;color:#fff;padding:6px 9px;text-align:left}
td{padding:5px 9px;border-bottom:1px solid #eee}.big{font-size:1.2em;font-weight:bold;color:#1565C0}
canvas{display:block;margin:10px auto}.note{color:#666;font-size:.86em}.neg{color:#c62828}.pos{color:#2e7d32}
</style></head><body>
<h1>DD 残差 vs 材质/几何</h1>
<div class="card"><p>NLOS 样本（反射∩DD）: <span class="big" id="n"></span></p>
<table><tr><th>子集</th><th>n</th><th>dd~ΔL Spearman</th><th>Pearson</th><th>拟合斜率</th><th>dd~ρ Spearman</th></tr>
<tbody id="tb"></tbody></table>
<p class="note">几何理想：dd~ΔL 强正相关、斜率≈1（伪距偏差=反射额外路径）</p>
<p class="note" id="gp"></p></div>
<div class="card"><b>dd_resid vs ΔL 散点（红线=理想 y=x）</b>
<canvas id="sc" width="700" height="500"></canvas></div>
<script>
const D=__PAGE_DATA__;
document.getElementById('n').textContent=D.n;
const tb=document.getElementById('tb');
D.results.forEach(r=>{const tr=document.createElement('tr');
  const cls=r.sp_dL>0.2?'pos':(r.sp_dL<-0.1?'neg':'');
  tr.innerHTML='<td>'+r.label+'</td><td>'+r.n+'</td><td class="'+cls+'">'+r.sp_dL
    +'</td><td>'+r.pe_dL+'</td><td>'+r.slope+'</td><td>'+r.sp_rho+'</td>';
  tb.appendChild(tr);});
if(D.gps_partial.r!==null)
  document.getElementById('gp').textContent='GPS-only 偏相关 dd~log(ρ)|ΔL: r='
    +D.gps_partial.r+' (p='+D.gps_partial.p+') → '
    +(D.gps_partial.p<0.05?'材质有增量贡献':'材质无显著增量');
(function(){const cv=document.getElementById('sc'),ctx=cv.getContext('2d');
  const W=cv.width,H=cv.height,p=50,data=D.scatter;
  const xs=data.map(d=>d.dL),ys=data.map(d=>d.dd);
  const xMax=Math.max.apply(null,xs)*1.05;
  const yLo=Math.min.apply(null,ys),yHi=Math.max.apply(null,ys);
  const lo=Math.min(0,yLo),hi=Math.max(xMax,yHi);
  const sx=v=>p+v/xMax*(W-2*p),sy=v=>H-p-(v-lo)/(hi-lo)*(H-2*p);
  ctx.fillStyle='#fafafa';ctx.fillRect(0,0,W,H);
  ctx.strokeStyle='#c62828';ctx.lineWidth=2;ctx.setLineDash([6,4]);
  ctx.beginPath();ctx.moveTo(sx(0),sy(0));ctx.lineTo(sx(xMax),sy(xMax));ctx.stroke();ctx.setLineDash([]);
  const SC={mild:'#42A5F5',strong:'#FFA726',severe:'#EF5350'};
  data.forEach(d=>{ctx.beginPath();ctx.arc(sx(d.dL),sy(d.dd),2.5,0,2*Math.PI);
    ctx.fillStyle=(SC[d.sev]||'#888')+'aa';ctx.fill();});
  ctx.strokeStyle='#333';ctx.lineWidth=1.5;
  ctx.beginPath();ctx.moveTo(p,p);ctx.lineTo(p,H-p);ctx.lineTo(W-p,H-p);ctx.stroke();
  ctx.fillStyle='#333';ctx.font='12px Arial';ctx.textAlign='center';
  ctx.fillText('ΔL 预测额外路径 (m)',W/2,H-8);
  ctx.save();ctx.translate(14,H/2);ctx.rotate(-Math.PI/2);
  ctx.fillText('DD 残差 (m)',0,0);ctx.restore();})();
</script></body></html>"""
html = html.replace('__PAGE_DATA__', json.dumps(page))
with open(args.out_html, 'w') as f:
    f.write(html)
print(f'写出 HTML: {args.out_html}')

# ── 核心结论 ─────────────────────────────────────────────────────────────────
overall = results[0]
gps = next((r for r in results if r['label']=='星座 G'), None)
print(f'\n=== 核心结论 ===')
print(f'  全样本 dd~ΔL: Spearman={overall["sp_dL"]:+.3f}  斜率={overall["slope"]:.2f}')
if gps:
    print(f'  GPS-only dd~ΔL: Spearman={gps["sp_dL"]:+.3f}  斜率={gps["slope"]:.2f}')
if overall['sp_dL'] > 0.3:
    print(f'  → ✅ DD 验证 NLOS 反射模型：伪距偏差由 ΔL 主导')
else:
    print(f'  → ⚠️ DD 也未显示 ΔL 主导，可能 1-bounce ΔL 模型不足或反射几何噪声大')
print(f'\n拷出: sudo docker cp ros1_gnss:{args.out_html} ~/dd_resid_material.html')
