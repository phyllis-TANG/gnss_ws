#!/usr/bin/env python3
"""
lidar_step6e_verify_material.py
材质效应"死活验证"——判断 ρ_norm vs CN0_drop 的弱相关 (r≈-0.066) 属于：
  情况 A（样本不足）：真实关联存在，但被采样噪声淹没 → 加数据有用
  情况 B（物理噪声地板）：关联本身极弱，CN0 噪声 >> 材质信号 → 加数据无用

三种诊断（全部在现有 rho_cn0_analysis.csv 上，不需新数据）：
  1. Bootstrap 置信区间：r 的 95% CI 是否跨过 0
  2. 分层相关性：按 severity / hit_dist / incidence 分层，子集内 r 是否一致为负
  3. 偏相关：控制 elevation + hit_dist 后，ρ_norm 对 CN0_drop 的净贡献

输出决策：A / B / 边界，并给出建议（是否值得扩展多星座数据）

输入:
  --data  rho_cn0_analysis.csv   (step6c 输出)
输出:
  --out_csv   material_verify.csv    (分层结果表)
  --out_html  material_verify.html   (决策报告)

用法:
  python3 lidar_step6e_verify_material.py \\
    --data /root/rho_cn0_analysis.csv \\
    --out_csv  /root/material_verify.csv \\
    --out_html /root/material_verify.html
"""

import argparse, csv, json, math
import numpy as np
from scipy import stats

ap = argparse.ArgumentParser()
ap.add_argument('--data',     default='/root/rho_cn0_analysis.csv')
ap.add_argument('--out_csv',  default='/root/material_verify.csv')
ap.add_argument('--out_html', default='/root/material_verify.html')
ap.add_argument('--n_boot',   type=int, default=10000)
ap.add_argument('--seed',     type=int, default=42)
args = ap.parse_args()

rng = np.random.default_rng(args.seed)

# ── 读数据 ────────────────────────────────────────────────────────────────────
print('读取数据...')
rho, drop, elev, dist, inc, sev = [], [], [], [], [], []
with open(args.data) as f:
    for r in csv.DictReader(f):
        try:
            if r.get('rho_norm', '') == '' or r.get('cn0_drop', '') == '':
                continue
            rho.append(float(r['rho_norm']))
            drop.append(float(r['cn0_drop']))
            elev.append(float(r['elevation']))
            dist.append(float(r['hit_dist_m']))
            inc.append(float(r['incidence_deg']) if r.get('incidence_deg') else 45.0)
            sev.append(r.get('severity', 'strong'))
        except (ValueError, KeyError):
            continue

rho  = np.array(rho);  drop = np.array(drop)
elev = np.array(elev); dist = np.array(dist); inc = np.array(inc)
sev  = np.array(sev)
n = len(rho)
print(f'  有效样本: {n}')
print(f'  CN0_drop: 均值={drop.mean():.2f}  std={drop.std():.2f} dB')
print(f'  ρ_norm:   中位={np.median(rho):.2f}  范围=[{rho.min():.2f}, {rho.max():.2f}]')

# log(ρ) 用于线性相关（ρ 跨数量级）
log_rho = np.log(rho)

# ── 诊断 1: Bootstrap 置信区间 ────────────────────────────────────────────────
print(f'\n── 诊断 1: Bootstrap 置信区间 (n_boot={args.n_boot}) ──')

def boot_ci(x, y, kind='spearman'):
    stat_fn = (lambda a, b: stats.spearmanr(a, b)[0]) if kind == 'spearman' \
              else (lambda a, b: stats.pearsonr(a, b)[0])
    point = stat_fn(x, y)
    boots = np.empty(args.n_boot)
    idx_all = np.arange(len(x))
    for i in range(args.n_boot):
        idx = rng.choice(idx_all, size=len(x), replace=True)
        boots[i] = stat_fn(x[idx], y[idx])
    lo, hi = np.percentile(boots, [2.5, 97.5])
    # 与 0 同号的 bootstrap 比例（单边证据强度）
    frac_neg = float(np.mean(boots < 0))
    return point, lo, hi, frac_neg, boots

sp_point, sp_lo, sp_hi, sp_fracneg, sp_boots = boot_ci(rho, drop, 'spearman')
pe_point, pe_lo, pe_hi, pe_fracneg, _        = boot_ci(log_rho, drop, 'pearson')

print(f'  Spearman r = {sp_point:+.4f}   95% CI = [{sp_lo:+.4f}, {sp_hi:+.4f}]')
print(f'    bootstrap 中 r<0 的比例 = {100*sp_fracneg:.1f}%')
print(f'  Pearson(logρ) r = {pe_point:+.4f}   95% CI = [{pe_lo:+.4f}, {pe_hi:+.4f}]')
print(f'    bootstrap 中 r<0 的比例 = {100*pe_fracneg:.1f}%')

ci_excludes_zero = (sp_lo < 0 and sp_hi < 0)
print(f'  → Spearman 95% CI {"排除" if ci_excludes_zero else "包含"} 0 '
      f'（{"显著" if ci_excludes_zero else "不显著"}）')

# ── 诊断 2: 分层相关性 ────────────────────────────────────────────────────────
print(f'\n── 诊断 2: 分层相关性 ──')
strata_results = []

def add_stratum(group, label, mask):
    if mask.sum() < 20:
        return
    rs, ps = stats.spearmanr(rho[mask], drop[mask])
    strata_results.append({
        'group': group, 'label': label, 'n': int(mask.sum()),
        'r': float(rs), 'p': float(ps),
        'drop_mean': float(drop[mask].mean()),
        'rho_med': float(np.median(rho[mask])),
    })

# 按 severity
print('  [按 severity]')
for s in ['mild', 'strong', 'severe']:
    add_stratum('severity', s, sev == s)
# 按 hit_dist 三分位
print('  [按 hit_dist 三分位]')
d_q = np.percentile(dist, [33.3, 66.7])
add_stratum('hit_dist', f'近 (<{d_q[0]:.0f}m)',        dist < d_q[0])
add_stratum('hit_dist', f'中 ({d_q[0]:.0f}-{d_q[1]:.0f}m)', (dist >= d_q[0]) & (dist < d_q[1]))
add_stratum('hit_dist', f'远 (>{d_q[1]:.0f}m)',        dist >= d_q[1])
# 按入射角三分位
print('  [按 incidence 三分位]')
i_q = np.percentile(inc, [33.3, 66.7])
add_stratum('incidence', f'小 (<{i_q[0]:.0f}°)',        inc < i_q[0])
add_stratum('incidence', f'中 ({i_q[0]:.0f}-{i_q[1]:.0f}°)', (inc >= i_q[0]) & (inc < i_q[1]))
add_stratum('incidence', f'大 (>{i_q[1]:.0f}°)',        inc >= i_q[1])

for r in strata_results:
    sig = '*' if r['p'] < 0.05 else ' '
    print(f"    {r['group']:9s} {r['label']:16s} n={r['n']:4d}  "
          f"r={r['r']:+.3f}{sig}  p={r['p']:.3f}  "
          f"drop均值={r['drop_mean']:+.2f}dB")

# 一致性指标：多少比例的分层 r 为负
n_strata = len(strata_results)
n_neg    = sum(1 for r in strata_results if r['r'] < 0)
n_neg_sig = sum(1 for r in strata_results if r['r'] < 0 and r['p'] < 0.05)
print(f'  → {n_neg}/{n_strata} 个分层 r<0，其中 {n_neg_sig} 个显著 (p<0.05)')

# ── 诊断 3: 偏相关（控制 elevation + hit_dist）────────────────────────────────
print(f'\n── 诊断 3: 偏相关（控制 elevation + hit_dist）──')

def residualize(y, *covariates):
    """回归剔除协变量后的残差"""
    X = np.column_stack([np.ones(len(y))] + list(covariates))
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    return y - X @ beta

sin_e = np.sin(np.radians(elev))
# 剔除 elevation（sin形式）和 log(hit_dist)
log_d = np.log(dist)
rho_resid  = residualize(log_rho, sin_e, log_d)
drop_resid = residualize(drop,    sin_e, log_d)

pcorr, p_pcorr = stats.pearsonr(rho_resid, drop_resid)
print(f'  原始 Pearson(logρ, drop)      = {pe_point:+.4f}')
print(f'  偏相关(logρ, drop | elev,dist) = {pcorr:+.4f}  p={p_pcorr:.4f}')
preserved = abs(pcorr) >= 0.5 * abs(pe_point) and (pcorr * pe_point > 0)
print(f'  → 控制混淆后效应 {"保留" if preserved else "消失/反转"}'
      f'（{"材质有独立贡献" if preserved else "可能是混淆驱动"}）')

# 方差解释（信噪比框架）
var_total = drop.var()
var_resid_after_rho = residualize(drop, log_rho).var()
r2_rho_alone = 1 - var_resid_after_rho / var_total
print(f'\n  ρ_norm 单独解释的 CN0_drop 方差占比 = {100*r2_rho_alone:.2f}%')
print(f'  （CN0_drop 总方差 = {var_total:.2f} dB²，材质信号埋在 {drop.std():.1f}dB 噪声中）')

# ── 综合决策 ──────────────────────────────────────────────────────────────────
print(f'\n{"="*60}')
print('综合决策')
print(f'{"="*60}')

score_A = 0  # 倾向"样本不足，加数据有用"的证据
notes = []
if ci_excludes_zero:
    score_A += 1; notes.append('✓ Bootstrap CI 排除 0（关联显著为负）')
else:
    notes.append('✗ Bootstrap CI 包含 0（整体不显著）')
if n_neg >= 0.7 * n_strata:
    score_A += 1; notes.append(f'✓ 分层 r 高度一致为负 ({n_neg}/{n_strata})')
else:
    notes.append(f'✗ 分层 r 不一致 ({n_neg}/{n_strata} 为负）')
if n_neg_sig >= 2:
    score_A += 1; notes.append(f'✓ 有 {n_neg_sig} 个分层显著')
else:
    notes.append(f'✗ 仅 {n_neg_sig} 个分层显著')
if preserved:
    score_A += 1; notes.append('✓ 偏相关保留（材质有独立贡献）')
else:
    notes.append('✗ 偏相关消失（可能是混淆驱动）')

for nt in notes:
    print(f'  {nt}')

if score_A >= 3:
    decision = 'A'
    verdict  = '情况 A：真实但弱的关联，被样本噪声部分淹没'
    advice   = '✅ 值得扩展多星座数据（F9P 自带 BDS/Galileo）以收窄 CI、提升统计功效'
elif score_A <= 1:
    decision = 'B'
    verdict  = '情况 B：物理噪声地板主导，材质信号 << CN0 噪声'
    advice   = ('⚠️ 加数据难以提升 r。但"噪声地板限制"本身是可发表的诚实结论。'
                '建议转向：用分层后最强的子集（如 severe）做定性论证，或转做 CN0 加权 SPP 闭环。')
else:
    decision = '边界'
    verdict  = '边界情况：证据混合，需谨慎'
    advice   = ('🔶 建议先做最便宜的扩展（F9P 多星座，不换接收机），'
                '在 ~2 倍样本上复测；若 severe 子集 r 稳定 <-0.15 则继续。')

print(f'\n  决策: {decision}')
print(f'  判断: {verdict}')
print(f'  建议: {advice}')

# ── 写 CSV ────────────────────────────────────────────────────────────────────
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['group','label','n','r','p','drop_mean','rho_med'])
    w.writeheader()
    for r in strata_results:
        w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})
print(f'\n写出 CSV: {args.out_csv}')

# ── 写 HTML ───────────────────────────────────────────────────────────────────
page = {
    'n': n,
    'drop_mean': round(float(drop.mean()), 2),
    'drop_std':  round(float(drop.std()), 2),
    'boot': {
        'sp_point': round(sp_point, 4), 'sp_lo': round(sp_lo, 4),
        'sp_hi': round(sp_hi, 4), 'sp_fracneg': round(sp_fracneg, 3),
        'pe_point': round(pe_point, 4), 'pe_lo': round(pe_lo, 4),
        'pe_hi': round(pe_hi, 4),
        'excludes_zero': bool(ci_excludes_zero),
        'hist': np.histogram(sp_boots, bins=40)[0].tolist(),
        'hist_edges': [round(float(e), 4) for e in np.histogram(sp_boots, bins=40)[1].tolist()],
    },
    'strata': strata_results,
    'strata_summary': {'n_strata': n_strata, 'n_neg': n_neg, 'n_neg_sig': n_neg_sig},
    'partial': {
        'raw': round(pe_point, 4), 'pcorr': round(float(pcorr), 4),
        'p': round(float(p_pcorr), 4), 'preserved': bool(preserved),
        'r2_rho': round(float(r2_rho_alone), 4),
    },
    'decision': decision, 'verdict': verdict, 'advice': advice,
    'notes': notes, 'score_A': score_A,
}

html = """<!DOCTYPE html>
<html lang="zh"><head><meta charset="UTF-8">
<title>材质效应验证报告</title>
<style>
  body{font-family:Arial,sans-serif;max-width:1050px;margin:20px auto;background:#f4f5f7;color:#333}
  h1{color:#1565C0;border-bottom:3px solid #1565C0;padding-bottom:8px}
  h2{color:#283593;margin-top:28px}
  .card{background:#fff;border-radius:8px;padding:18px;box-shadow:0 1px 5px rgba(0,0,0,.1);margin-bottom:16px}
  table{border-collapse:collapse;width:100%}
  th{background:#1565C0;color:#fff;padding:7px 10px;text-align:left}
  td{padding:6px 10px;border-bottom:1px solid #eee}
  tr:hover td{background:#f0f4ff}
  .neg{color:#c62828;font-weight:bold} .pos{color:#2e7d32;font-weight:bold}
  .note{color:#666;font-size:.88em;margin-top:6px}
  .verdict{font-size:1.15em;padding:16px;border-radius:8px;font-weight:bold}
  .v-A{background:#e8f5e9;color:#2e7d32;border-left:6px solid #2e7d32}
  .v-B{background:#fff3e0;color:#e65100;border-left:6px solid #e65100}
  .v-边界{background:#fffde7;color:#f9a825;border-left:6px solid #f9a825}
  ul.notes li{margin:4px 0}
  canvas{display:block;margin:8px auto}
</style></head><body>
<h1>材质效应"死活验证"报告</h1>
<p>判断 ρ_norm vs CN0_drop 的弱相关属于「样本不足(A)」还是「物理噪声地板(B)」</p>

<div class="card">
<div id="verdict_box" class="verdict"></div>
<p style="margin-top:12px"><b>建议：</b><span id="advice_text"></span></p>
<ul class="notes" id="notes_list"></ul>
<p class="note">样本数 n=<span id="n_val"></span>，CN0_drop 均值=<span id="dm_val"></span>dB，std=<span id="ds_val"></span>dB</p>
</div>

<h2>诊断 1：Bootstrap 置信区间</h2>
<div class="card">
<table>
<tr><th>统计量</th><th>点估计</th><th>95% CI</th><th>结论</th></tr>
<tr id="boot_sp"></tr>
<tr id="boot_pe"></tr>
</table>
<canvas id="boot_canvas" width="700" height="240"></canvas>
<p class="note">直方图为 Spearman r 的 bootstrap 分布；红虚线=0，蓝实线=点估计。
分布整体落在 0 左侧 → 关联稳定为负。</p>
</div>

<h2>诊断 2：分层相关性</h2>
<div class="card">
<p>非单调双体制下，混合所有样本算单一 r 会稀释效应。分层后子集内 r 是否一致为负？</p>
<table>
<tr><th>分层维度</th><th>区间</th><th>n</th><th>Spearman r</th><th>p</th><th>CN0_drop均值</th></tr>
<tbody id="strata_body"></tbody>
</table>
<p class="note" id="strata_summary"></p>
</div>

<h2>诊断 3：偏相关（控制混淆变量）</h2>
<div class="card">
<table>
<tr><th>相关性</th><th>r</th><th>说明</th></tr>
<tr id="pc_raw"></tr>
<tr id="pc_partial"></tr>
</table>
<p class="note">偏相关 = 同时剔除 elevation 和 hit_dist 后，ρ_norm 对 CN0_drop 的净贡献。
若接近原始相关 → 材质有独立物理贡献；若归零 → 之前的相关是混淆驱动的假象。</p>
<p class="note">ρ_norm 单独解释的 CN0_drop 方差占比：<b id="r2_val"></b></p>
</div>

<script>
const D = __PAGE_DATA__;
const $ = function(id){return document.getElementById(id);};

// ── 结论框 ──
$('verdict_box').className = 'verdict v-' + D.decision;
$('verdict_box').textContent = '决策 ' + D.decision + '：' + D.verdict;
$('advice_text').textContent = D.advice;
$('n_val').textContent = D.n;
$('dm_val').textContent = D.drop_mean;
$('ds_val').textContent = D.drop_std;
const ul = $('notes_list');
D.notes.forEach(function(nt){
  const li = document.createElement('li');
  li.textContent = nt;
  li.style.color = nt.indexOf('✓')>=0 ? '#2e7d32' : '#c62828';
  ul.appendChild(li);
});

// ── 诊断1 表 ──
const b = D.boot;
$('boot_sp').innerHTML = '<td>Spearman (ρ_norm, CN0_drop)</td>'
  +'<td class="'+(b.sp_point<0?'neg':'pos')+'">'+b.sp_point+'</td>'
  +'<td>['+b.sp_lo+', '+b.sp_hi+']</td>'
  +'<td>'+(b.excludes_zero?'<span class="neg">CI 排除 0 → 显著</span>':'<span style="color:#e65100">CI 含 0 → 不显著</span>')+'</td>';
$('boot_pe').innerHTML = '<td>Pearson (log ρ_norm, CN0_drop)</td>'
  +'<td class="'+(b.pe_point<0?'neg':'pos')+'">'+b.pe_point+'</td>'
  +'<td>['+b.pe_lo+', '+b.pe_hi+']</td>'
  +'<td>'+(100*b.sp_fracneg).toFixed(1)+'% 的 bootstrap r<0</td>';

// ── 诊断1 直方图 ──
(function(){
  const cv = $('boot_canvas'), ctx = cv.getContext('2d');
  const W=cv.width, H=cv.height, p=40;
  const hist = b.hist, edges = b.hist_edges;
  const xMin = edges[0], xMax = edges[edges.length-1];
  const yMax = Math.max.apply(null, hist);
  const sx = function(v){return p+(v-xMin)/(xMax-xMin)*(W-2*p);};
  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  const bw = (W-2*p)/hist.length;
  hist.forEach(function(c,i){
    const x = p+i*bw, bh=(H-2*p)*c/yMax;
    ctx.fillStyle='#1E88E5cc'; ctx.fillRect(x, H-p-bh, bw-1, bh);
  });
  // 0 线
  if(xMin<0 && xMax>0){
    const x0=sx(0);
    ctx.strokeStyle='#c62828'; ctx.lineWidth=2; ctx.setLineDash([5,4]);
    ctx.beginPath(); ctx.moveTo(x0,p); ctx.lineTo(x0,H-p); ctx.stroke();
    ctx.setLineDash([]);
  }
  // 点估计线
  const xp=sx(b.sp_point);
  ctx.strokeStyle='#1565C0'; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(xp,p); ctx.lineTo(xp,H-p); ctx.stroke();
  // 轴
  ctx.strokeStyle='#333'; ctx.lineWidth=1.2;
  ctx.beginPath(); ctx.moveTo(p,H-p); ctx.lineTo(W-p,H-p); ctx.stroke();
  ctx.fillStyle='#333'; ctx.font='11px Arial'; ctx.textAlign='center';
  ctx.fillText('Spearman r', W/2, H-6);
  ctx.fillText(xMin.toFixed(2), p, H-p+14);
  ctx.fillText(xMax.toFixed(2), W-p, H-p+14);
  ctx.fillStyle='#c62828'; ctx.textAlign='left';
  ctx.fillText('r=0', sx(0)+3, p+12);
})();

// ── 诊断2 分层表 ──
const tb = $('strata_body');
let lastGroup = '';
D.strata.forEach(function(s){
  const tr = document.createElement('tr');
  const grpCell = s.group !== lastGroup ? '<td><b>'+s.group+'</b></td>' : '<td></td>';
  lastGroup = s.group;
  const sig = s.p < 0.05 ? ' *' : '';
  tr.innerHTML = grpCell
    +'<td>'+s.label+'</td><td>'+s.n+'</td>'
    +'<td class="'+(s.r<0?'neg':'pos')+'">'+s.r.toFixed(3)+sig+'</td>'
    +'<td>'+s.p.toFixed(3)+'</td>'
    +'<td>'+s.drop_mean.toFixed(2)+' dB</td>';
  tb.appendChild(tr);
});
const ss = D.strata_summary;
$('strata_summary').innerHTML = '共 '+ss.n_strata+' 个分层，其中 <b>'+ss.n_neg
  +'</b> 个 r<0，<b>'+ss.n_neg_sig+'</b> 个显著 (p<0.05，标 *)';

// ── 诊断3 偏相关 ──
const pc = D.partial;
$('pc_raw').innerHTML = '<td>原始 Pearson(logρ, drop)</td>'
  +'<td class="'+(pc.raw<0?'neg':'pos')+'">'+pc.raw+'</td><td>未控制混淆</td>';
$('pc_partial').innerHTML = '<td>偏相关 (logρ, drop | elev, dist)</td>'
  +'<td class="'+(pc.pcorr<0?'neg':'pos')+'">'+pc.pcorr+' (p='+pc.p+')</td>'
  +'<td>'+(pc.preserved?'<span class="pos">效应保留 → 材质有独立贡献</span>'
                       :'<span class="neg">效应消失 → 可能混淆驱动</span>')+'</td>';
$('r2_val').textContent = (100*pc.r2_rho).toFixed(2) + '%';
</script>
</body></html>"""

html = html.replace('__PAGE_DATA__', json.dumps(page))
with open(args.out_html, 'w') as f:
    f.write(html)
print(f'写出 HTML: {args.out_html}')

print(f'\n=== 完成 ===')
print(f'决策: {decision} — {verdict}')
print(f'\n拷出: sudo docker cp ros1_gnss:{args.out_html} ~/material_verify.html')
