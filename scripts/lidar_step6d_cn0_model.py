#!/usr/bin/env python3
"""
lidar_step6d_cn0_model.py
多变量 CN0_drop 预测模型（物理驱动线性回归 + 随机森林）

物理基础（Friis 传输方程 + LiDAR 方程）：
    CN0_drop ≈ α₁·log(R) − α₂·log(ρ) + α₃·log(1/cos θ) + f(severity)
               路径损耗       材质项       入射角项

    理论预期系数符号：α₁>0（路径越远衰减越大），α₂>0（反射率越高衰减越小），
                      α₃>0（掠射角越大衰减越大）
    若线性模型系数符号与理论一致，则支持物理模型假设。

两种模型：
  1. 物理线性回归（PhysLR）：对数变换特征，系数可直接对应物理量
  2. 随机森林（RF）：非线性，特征重要性排名，代表性能上界

时序划分：按 epoch 排序，前 70% 训练，后 30% 测试（避免时间泄漏）

输入:
  --data  rho_cn0_analysis.csv   (step6c 输出)

输出:
  --out_csv   cn0_model_results.csv   (逐行预测值+残差)
  --out_html  cn0_model_report.html   (可视化报告)

用法:
  python3 lidar_step6d_cn0_model.py \\
    --data     /root/rho_cn0_analysis.csv \\
    --out_csv  /root/cn0_model_results.csv \\
    --out_html /root/cn0_model_report.html
"""

import argparse, csv, math, json, warnings
import numpy as np
from sklearn.linear_model import Ridge
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
import scipy.stats as sps

warnings.filterwarnings('ignore')

ap = argparse.ArgumentParser()
ap.add_argument('--data',      default='/root/rho_cn0_analysis.csv')
ap.add_argument('--out_csv',   default='/root/cn0_model_results.csv')
ap.add_argument('--out_html',  default='/root/cn0_model_report.html')
ap.add_argument('--test_frac', type=float, default=0.3)
args = ap.parse_args()

# ── 读数据 ────────────────────────────────────────────────────────────────────
print('读取数据...')
rows = []
with open(args.data) as f:
    for r in csv.DictReader(f):
        try:
            rho  = float(r['rho_norm'])
            dist = float(r['hit_dist_m'])
            inc  = float(r['incidence_deg']) if r.get('incidence_deg') else 45.0
            elev = float(r['elevation'])
            drop = float(r['cn0_drop'])
            cn0  = float(r['cn0'])
            sev  = r.get('severity', 'strong')
            epch = float(r['epoch'])
            if rho <= 0 or dist <= 0 or math.isnan(drop):
                continue
            rows.append({'epoch': epch, 'sat_id': r.get('sat_id',''),
                         'rho_norm': rho, 'hit_dist_m': dist,
                         'incidence_deg': inc, 'elevation': elev,
                         'cn0_drop': drop, 'cn0': cn0, 'severity': sev})
        except (ValueError, KeyError):
            pass

print(f'  有效样本: {len(rows)}')
for s in ['mild', 'strong', 'severe']:
    print(f'    {s}: {sum(1 for r in rows if r["severity"]==s)}')

# ── 特征工程 ──────────────────────────────────────────────────────────────────
# 物理变换：对应方程各项
# PhysLR 特征: [log(ρ), log(R), -log(cos α), sin(elev), severity_code]
# RF 特征: 原始 + 变换（让模型自选）
SEV_CODE = {'mild': 0, 'strong': 1, 'severe': 2}
PHYS_FEAT_NAMES = ['log(ρ_norm)', 'log(R_hit)', '-log(cos α)', 'sin(elev)', 'severity']
RF_FEAT_NAMES   = ['ρ_norm', 'R(m)', 'α(°)', 'elev(°)', 'severity',
                   'log(ρ)', 'log(R)', '-log(cosα)']
# 理论系数符号: log_rho→负（高ρ→小drop）, log_R→正, neg_log_cos→正, sin_elev→负, sev→正
EXPECTED_SIGNS  = [-1, +1, +1, -1, +1]

def make_features(batch):
    Xp, Xr, y = [], [], []
    for r in batch:
        rho  = r['rho_norm']
        dist = r['hit_dist_m']
        inc  = r['incidence_deg']
        elev = r['elevation']
        sev  = SEV_CODE.get(r['severity'], 1)

        cos_inc    = max(math.cos(math.radians(inc)), 0.05)
        log_rho    =  math.log(rho)
        log_dist   =  math.log(dist)
        neg_log_cos = -math.log(cos_inc)
        sin_elev   =  math.sin(math.radians(elev))

        Xp.append([log_rho, log_dist, neg_log_cos, sin_elev, sev])
        Xr.append([rho, dist, inc, elev, sev, log_rho, log_dist, neg_log_cos])
        y.append(r['cn0_drop'])
    return np.array(Xp), np.array(Xr), np.array(y)

# ── 时序划分 ──────────────────────────────────────────────────────────────────
rows_sorted = sorted(rows, key=lambda r: r['epoch'])
n      = len(rows_sorted)
n_test = int(n * args.test_frac)
n_tr   = n - n_test
tr_rows, te_rows = rows_sorted[:n_tr], rows_sorted[n_tr:]

print(f'\n时序划分: 训练 {n_tr}, 测试 {n_test}')
print(f'  训练 epoch: [{tr_rows[0]["epoch"]:.1f}, {tr_rows[-1]["epoch"]:.1f}]')
print(f'  测试 epoch: [{te_rows[0]["epoch"]:.1f}, {te_rows[-1]["epoch"]:.1f}]')

Xp_tr, Xr_tr, y_tr = make_features(tr_rows)
Xp_te, Xr_te, y_te = make_features(te_rows)

# ── 模型1: 物理线性回归 ───────────────────────────────────────────────────────
print('\n── 模型1: 物理线性回归 (Ridge) ──')
lr = Ridge(alpha=0.1)
lr.fit(Xp_tr, y_tr)
yp_lr_tr = lr.predict(Xp_tr)
yp_lr_te = lr.predict(Xp_te)

r2_lr_tr = r2_score(y_tr, yp_lr_tr)
r2_lr_te = r2_score(y_te, yp_lr_te)
mae_lr   = mean_absolute_error(y_te, yp_lr_te)
rmse_lr  = math.sqrt(mean_squared_error(y_te, yp_lr_te))
print(f'  训练 R² = {r2_lr_tr:.4f}')
print(f'  测试  R² = {r2_lr_te:.4f}   MAE = {mae_lr:.3f} dB   RMSE = {rmse_lr:.3f} dB')

print(f'\n  {"特征":<16} {"系数":>10}  {"理论":>6}  {"符合":>5}')
for nm, coef, exp in zip(PHYS_FEAT_NAMES, lr.coef_, EXPECTED_SIGNS):
    ok = '✓' if coef * exp > 0 else '✗'
    print(f'  {nm:<16} {coef:>+10.4f}  {"+" if exp>0 else "-":>6}  {ok:>5}')
print(f'  截距: {lr.intercept_:.4f}')

# ── 模型2: 随机森林 ───────────────────────────────────────────────────────────
print('\n── 模型2: 随机森林 ──')
rf = RandomForestRegressor(n_estimators=300, max_depth=8, min_samples_leaf=8,
                            random_state=42, n_jobs=-1)
rf.fit(Xr_tr, y_tr)
yp_rf_tr = rf.predict(Xr_tr)
yp_rf_te = rf.predict(Xr_te)

r2_rf_tr = r2_score(y_tr, yp_rf_tr)
r2_rf_te = r2_score(y_te, yp_rf_te)
mae_rf   = mean_absolute_error(y_te, yp_rf_te)
rmse_rf  = math.sqrt(mean_squared_error(y_te, yp_rf_te))
print(f'  训练 R² = {r2_rf_tr:.4f}')
print(f'  测试  R² = {r2_rf_te:.4f}   MAE = {mae_rf:.3f} dB   RMSE = {rmse_rf:.3f} dB')

importances = rf.feature_importances_
imp_idx = np.argsort(importances)[::-1]
print('\n  特征重要性:')
for i in imp_idx:
    print(f'    {RF_FEAT_NAMES[i]:<14}: {importances[i]:.4f}  ({100*importances[i]:.1f}%)')

# ── Baseline: 仅用 severity 均值 ──────────────────────────────────────────────
sev_mean = {s: float(np.mean([r['cn0_drop'] for r in tr_rows if r['severity']==s]))
            for s in ['mild','strong','severe']}
global_mean = float(np.mean(y_tr))
yp_base_te = np.array([sev_mean.get(r['severity'], global_mean) for r in te_rows])
r2_base  = r2_score(y_te, yp_base_te)
mae_base = mean_absolute_error(y_te, yp_base_te)
print(f'\n── Baseline (severity 均值) ──')
print(f'  测试 R² = {r2_base:.4f}   MAE = {mae_base:.3f} dB')
print(f'  LR vs Baseline  ΔR² = {r2_lr_te - r2_base:+.4f}  ΔMAE = {mae_lr - mae_base:+.3f}dB')
print(f'  RF vs Baseline  ΔR² = {r2_rf_te - r2_base:+.4f}  ΔMAE = {mae_rf - mae_base:+.3f}dB')

# ── 按 severity 测试集表现 ────────────────────────────────────────────────────
print(f'\n── 按 severity 分组（测试集）──')
print(f'  {"sev":<8} {"n":>5}  {"LR_R²":>7}  {"RF_R²":>7}  {"LR_MAE":>7}  {"RF_MAE":>7}')
sev_te = np.array([r['severity'] for r in te_rows])
for s in ['mild', 'strong', 'severe']:
    mask = sev_te == s
    if mask.sum() < 5: continue
    r2s_lr  = r2_score(y_te[mask], yp_lr_te[mask])
    r2s_rf  = r2_score(y_te[mask], yp_rf_te[mask])
    mae_slr = mean_absolute_error(y_te[mask], yp_lr_te[mask])
    mae_srf = mean_absolute_error(y_te[mask], yp_rf_te[mask])
    print(f'  {s:<8} {mask.sum():>5}  {r2s_lr:>7.4f}  {r2s_rf:>7.4f}  '
          f'{mae_slr:>7.3f}  {mae_srf:>7.3f}')

# ── 物理模型残差分析 ──────────────────────────────────────────────────────────
resid_te = y_te - yp_lr_te
print(f'\n── 线性模型残差（测试集）──')
print(f'  均值: {resid_te.mean():.3f} dB  std: {resid_te.std():.3f} dB')
print(f'  5th~95th: [{np.percentile(resid_te,5):.2f}, {np.percentile(resid_te,95):.2f}] dB')
_, p_norm = sps.normaltest(resid_te)
print(f'  残差正态检验 p = {p_norm:.3e}  (p>0.05 则接近正态)')

# ── 写 CSV ────────────────────────────────────────────────────────────────────
print(f'\n写出: {args.out_csv}')
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=[
        'epoch','sat_id','severity','split',
        'rho_norm','hit_dist_m','incidence_deg','elevation',
        'cn0','cn0_drop','pred_lr','pred_rf','resid_lr','resid_rf'])
    w.writeheader()
    for batch, yp_lr, yp_rf, split in [
            (tr_rows, yp_lr_tr, yp_rf_tr, 'train'),
            (te_rows, yp_lr_te, yp_rf_te, 'test')]:
        for r, pl, pf in zip(batch, yp_lr.tolist(), yp_rf.tolist()):
            w.writerow({
                'epoch':         f"{r['epoch']:.3f}",
                'sat_id':        r['sat_id'],
                'severity':      r['severity'],
                'split':         split,
                'rho_norm':      f"{r['rho_norm']:.4f}",
                'hit_dist_m':    f"{r['hit_dist_m']:.2f}",
                'incidence_deg': f"{r['incidence_deg']:.1f}",
                'elevation':     f"{r['elevation']:.1f}",
                'cn0':           f"{r['cn0']:.1f}",
                'cn0_drop':      f"{r['cn0_drop']:.3f}",
                'pred_lr':       f"{pl:.3f}",
                'pred_rf':       f"{pf:.3f}",
                'resid_lr':      f"{r['cn0_drop']-pl:.3f}",
                'resid_rf':      f"{r['cn0_drop']-pf:.3f}",
            })

# ── HTML 报告 ─────────────────────────────────────────────────────────────────
print(f'写出: {args.out_html}')

# 注入 JSON 数据（避免 Python f-string vs JS 模板字符串冲突）
SEV_COLOR = {'mild': '#42A5F5', 'strong': '#FFA726', 'severe': '#EF5350'}
page_data = {
    'summary': {
        'n_train': n_tr, 'n_test': n_test,
        'lr':   {'r2_tr': round(r2_lr_tr,4), 'r2_te': round(r2_lr_te,4),
                 'mae': round(mae_lr,3),     'rmse': round(rmse_lr,3)},
        'rf':   {'r2_tr': round(r2_rf_tr,4), 'r2_te': round(r2_rf_te,4),
                 'mae': round(mae_rf,3),     'rmse': round(rmse_rf,3)},
        'base': {'r2_te': round(r2_base,4),  'mae': round(mae_base,3)},
    },
    'coefs': [
        {'name': nm, 'coef': round(float(c),4), 'expected': int(e),
         'ok': bool(c * e > 0)}
        for nm, c, e in zip(PHYS_FEAT_NAMES, lr.coef_, EXPECTED_SIGNS)
    ],
    'intercept': round(float(lr.intercept_), 4),
    'importances': [
        {'name': RF_FEAT_NAMES[i], 'imp': round(float(importances[i]),4)}
        for i in imp_idx
    ],
    'scatter': [
        {'actual': round(float(y_te[i]),2),
         'pred_lr': round(float(yp_lr_te[i]),2),
         'pred_rf': round(float(yp_rf_te[i]),2),
         'sev': te_rows[i]['severity']}
        for i in range(len(y_te))
    ],
    'resid': [round(float(v),3) for v in resid_te.tolist()],
}

html = """<!DOCTYPE html>
<html lang="zh"><head>
<meta charset="UTF-8">
<title>CN0_drop 预测模型报告</title>
<style>
  body{font-family:Arial,sans-serif;max-width:1100px;margin:20px auto;background:#f5f5f5;color:#333}
  h1{color:#1565C0;border-bottom:2px solid #1565C0;padding-bottom:6px}
  h2{color:#283593;margin-top:28px}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin-bottom:20px}
  .card{background:#fff;border-radius:8px;padding:18px;box-shadow:0 1px 4px rgba(0,0,0,.12)}
  table{border-collapse:collapse;width:100%}
  th{background:#1565C0;color:#fff;padding:7px 10px;text-align:left}
  td{padding:6px 10px;border-bottom:1px solid #eee}
  tr:hover td{background:#f0f4ff}
  .ok{color:#2e7d32;font-weight:bold} .bad{color:#c62828;font-weight:bold}
  canvas{display:block;margin:0 auto}
  .note{color:#666;font-size:0.88em;margin-top:6px}
</style>
</head><body>
<h1>CN0_drop 预测模型报告</h1>
<p>基于物理驱动特征（LiDAR 反射率 + 几何参数）预测 GNSS NLOS 信号衰减量</p>

<h2>1. 模型性能对比（测试集）</h2>
<div class="card">
<table>
<tr><th>模型</th><th>测试 R²</th><th>训练 R²</th><th>MAE (dB)</th><th>RMSE (dB)</th><th>说明</th></tr>
<tr id="row_base"></tr>
<tr id="row_lr"></tr>
<tr id="row_rf"></tr>
</table>
<p class="note">时序划分: 前 70% epoch 训练，后 30% 测试。R² 越高越好，MAE 越低越好。</p>
</div>

<h2>2. 物理模型系数验证</h2>
<div class="card">
<p>理论：CN0_drop ≈ α₁·log(R) − α₂·log(ρ) + α₃·log(1/cos θ) + ...<br>
   系数符号与理论一致（✓）则支持物理假设。</p>
<table>
<tr><th>特征</th><th>拟合系数</th><th>理论符号</th><th>符合?</th><th>物理解释</th></tr>
<tbody id="coef_body"></tbody>
</table>
<p class="note">截距: <span id="intercept_val"></span> dB-Hz</p>
</div>

<h2>3. 随机森林特征重要性</h2>
<div class="card">
<canvas id="fi_canvas" width="700" height="300"></canvas>
<p class="note">条越长 → 该特征对 CN0_drop 预测贡献越大</p>
</div>

<h2>4. 实测 vs 预测散点图（测试集）</h2>
<div class="grid">
<div class="card">
  <b>物理线性模型 (PhysLR)</b>
  <canvas id="sc_lr" width="420" height="380"></canvas>
</div>
<div class="card">
  <b>随机森林 (RF)</b>
  <canvas id="sc_rf" width="420" height="380"></canvas>
</div>
</div>

<h2>5. 残差分布（物理线性模型，测试集）</h2>
<div class="card">
<canvas id="resid_canvas" width="700" height="260"></canvas>
<p class="note">残差 = 实测 − 预测。理想情况：零均值，近似正态分布。</p>
</div>

<script>
// ── 数据注入 ──
const D = __PAGE_DATA__;

// ── 工具函数 ──
function $(id){return document.getElementById(id);}
function pad(x,ctx,w,h,p){return {x:p,y:p,w:w-2*p,h:h-2*p};}

// ── 1. 性能表 ──
const s = D.summary;
function fmtRow(label, data, cls){
  return '<td>'+label+'</td><td class="'+cls+'">'+data.r2_te+'</td><td>'+data.r2_tr+'</td><td>'+data.mae+'</td>'
    +(data.rmse !== undefined ? '<td>'+data.rmse+'</td>' : '<td>—</td>');
}
$('row_base').innerHTML = fmtRow('Baseline (severity均值)', s.base, '') + '<td>仅用 severity 均值，无特征</td>';
$('row_lr').innerHTML   = fmtRow('物理线性 (PhysLR)',       s.lr,   s.lr.r2_te > s.base.r2_te ? 'ok':'bad') + '<td>log变换特征，Ridge回归</td>';
$('row_rf').innerHTML   = fmtRow('随机森林 (RF)',            s.rf,   s.rf.r2_te > s.base.r2_te ? 'ok':'bad') + '<td>300棵树，max_depth=8</td>';

// ── 2. 系数表 ──
const PHYS_EXPLAIN = {
  'log(ρ_norm)':  '高反射率 → 衰减减小',
  'log(R_hit)':   '路径越远 → 衰减增大',
  '-log(cos α)':  '掠射角越大 → 衰减增大',
  'sin(elev)':    '高仰角 → 基准CN0更高',
  'severity':     'mild→strong→severe 路径差增大',
};
$('intercept_val').textContent = D.intercept;
const tbody = $('coef_body');
D.coefs.forEach(function(c){
  const row = document.createElement('tr');
  row.innerHTML = '<td>'+c.name+'</td>'
    + '<td style="font-weight:bold">'+(c.coef>=0?'+':'')+c.coef+'</td>'
    + '<td>'+(c.expected>0?'+':'−')+'</td>'
    + '<td class="'+(c.ok?'ok':'bad')+'">'+(c.ok?'✓':'✗')+'</td>'
    + '<td>'+(PHYS_EXPLAIN[c.name]||'')+'</td>';
  tbody.appendChild(row);
});

// ── 3. 特征重要性柱状图 ──
(function(){
  const cv = $('fi_canvas'), ctx = cv.getContext('2d');
  const W=cv.width, H=cv.height, pad=50, barH=28, gap=6;
  const data = D.importances;
  const maxImp = data[0].imp;
  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  const colors = ['#1E88E5','#42A5F5','#64B5F6','#90CAF9','#BBDEFB','#FFA726','#FFB74D','#FFCC80'];
  data.forEach(function(d, i){
    const y  = pad + i*(barH+gap);
    const bw = (W - pad - 120) * d.imp / maxImp;
    ctx.fillStyle = colors[i % colors.length];
    ctx.fillRect(pad, y, bw, barH);
    ctx.fillStyle = '#333'; ctx.font = '12px Arial'; ctx.textAlign = 'right';
    ctx.fillText(d.name, pad-6, y+barH/2+5);
    ctx.textAlign = 'left';
    ctx.fillText((d.imp*100).toFixed(1)+'%', pad+bw+4, y+barH/2+5);
  });
  ctx.fillStyle='#333'; ctx.font='bold 13px Arial'; ctx.textAlign='center';
  ctx.fillText('特征重要性（随机森林）', W/2, H-6);
})();

// ── 4. 散点图（实测 vs 预测）──
function drawScatter(canvasId, key, r2val){
  const cv = $(canvasId), ctx = cv.getContext('2d');
  const W=cv.width, H=cv.height, p=45;
  const data = D.scatter;
  const vals = data.map(function(d){return [d.actual, d[key]];});
  const allV = vals.flat();
  const vMin = Math.floor(Math.min(...allV)) - 1;
  const vMax = Math.ceil(Math.max(...allV))  + 1;
  const sc = function(v){ return p + (v-vMin)/(vMax-vMin)*(W-2*p); };
  const sy = function(v){ return H-p - (v-vMin)/(vMax-vMin)*(H-2*p); };

  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  // grid
  ctx.strokeStyle='#eee'; ctx.lineWidth=0.8;
  for(let v=Math.ceil(vMin);v<=vMax;v+=2){
    ctx.beginPath(); ctx.moveTo(sc(v),p); ctx.lineTo(sc(v),H-p); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p,sy(v)); ctx.lineTo(W-p,sy(v)); ctx.stroke();
  }
  // diagonal
  ctx.strokeStyle='#999'; ctx.lineWidth=1.2; ctx.setLineDash([5,4]);
  ctx.beginPath(); ctx.moveTo(sc(vMin),sy(vMin)); ctx.lineTo(sc(vMax),sy(vMax)); ctx.stroke();
  ctx.setLineDash([]);
  // points
  const SEV_COL = {mild:'#42A5F5', strong:'#FFA726', severe:'#EF5350'};
  data.forEach(function(d){
    ctx.beginPath();
    ctx.arc(sc(d.actual), sy(d[key]), 3, 0, 2*Math.PI);
    ctx.fillStyle = (SEV_COL[d.sev]||'#888') + 'aa';
    ctx.fill();
  });
  // axes
  ctx.strokeStyle='#333'; ctx.lineWidth=1.5; ctx.setLineDash([]);
  ctx.beginPath(); ctx.moveTo(p,p); ctx.lineTo(p,H-p); ctx.lineTo(W-p,H-p); ctx.stroke();
  ctx.fillStyle='#333'; ctx.font='11px Arial'; ctx.textAlign='center';
  ctx.fillText('实测 CN0_drop (dB)', W/2, H-5);
  ctx.save(); ctx.translate(12,H/2); ctx.rotate(-Math.PI/2);
  ctx.fillText('预测 CN0_drop (dB)', 0, 0); ctx.restore();
  // R² label
  ctx.font='bold 13px Arial'; ctx.textAlign='right';
  ctx.fillText('R²='+r2val, W-p-4, p+16);
  // legend
  ['mild','strong','severe'].forEach(function(sv,i){
    const lx = p+8+i*80;
    ctx.beginPath(); ctx.arc(lx+6, H-p-14, 5, 0, 2*Math.PI);
    ctx.fillStyle=SEV_COL[sv]; ctx.fill();
    ctx.fillStyle='#555'; ctx.font='10px Arial'; ctx.textAlign='left';
    ctx.fillText(sv, lx+14, H-p-10);
  });
}
drawScatter('sc_lr', 'pred_lr', s.lr.r2_te);
drawScatter('sc_rf', 'pred_rf', s.rf.r2_te);

// ── 5. 残差直方图 ──
(function(){
  const cv = $('resid_canvas'), ctx = cv.getContext('2d');
  const W=cv.width, H=cv.height, p=45;
  const resid = D.resid;
  const rMin = Math.floor(Math.min(...resid))-1, rMax = Math.ceil(Math.max(...resid))+1;
  const nBins = 30, binW_val = (rMax-rMin)/nBins;
  const bins = new Array(nBins).fill(0);
  resid.forEach(function(v){
    const bi = Math.min(Math.floor((v-rMin)/binW_val), nBins-1);
    bins[bi]++;
  });
  const maxCount = Math.max(...bins);
  const bw = (W-2*p)/nBins;
  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  bins.forEach(function(cnt,i){
    const x  = p + i*bw;
    const bh = (H-2*p)*cnt/maxCount;
    ctx.fillStyle = '#1E88E5cc';
    ctx.fillRect(x, H-p-bh, bw-1, bh);
  });
  // zero line
  const x0 = p + (-rMin)/binW_val*bw;
  ctx.strokeStyle='#c62828'; ctx.lineWidth=1.5; ctx.setLineDash([4,3]);
  ctx.beginPath(); ctx.moveTo(x0,p); ctx.lineTo(x0,H-p); ctx.stroke();
  ctx.setLineDash([]);
  // axes
  ctx.strokeStyle='#333'; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(p,p); ctx.lineTo(p,H-p); ctx.lineTo(W-p,H-p); ctx.stroke();
  ctx.fillStyle='#333'; ctx.font='11px Arial'; ctx.textAlign='center';
  ctx.fillText('残差 (dB)', W/2, H-5);
  ctx.fillText('0', x0, H-p+13);
  // mean label
  const mean = resid.reduce(function(a,b){return a+b;},0)/resid.length;
  ctx.font='12px Arial'; ctx.textAlign='left';
  ctx.fillStyle='#c62828';
  ctx.fillText('均值='+mean.toFixed(3)+' dB', p+6, p+16);
})();

</script>
</body></html>"""

html = html.replace('__PAGE_DATA__', json.dumps(page_data))

with open(args.out_html, 'w') as f:
    f.write(html)

print('\n=== 完成 ===')
print(f'CSV:  {args.out_csv}')
print(f'HTML: {args.out_html}')
print(f'\n核心结论:')
print(f'  Baseline R²   = {r2_base:.4f}  (仅 severity 均值)')
print(f'  PhysLR  R²   = {r2_lr_te:.4f}  ({"+" if r2_lr_te>r2_base else ""}{r2_lr_te-r2_base:+.4f} vs baseline)')
print(f'  RF      R²   = {r2_rf_te:.4f}  ({"+' if r2_rf_te>r2_base else ""}{r2_rf_te-r2_base:+.4f} vs baseline)')
print(f'\n物理系数符号验证:')
for nm, coef, exp in zip(PHYS_FEAT_NAMES, lr.coef_, EXPECTED_SIGNS):
    ok = '✓ 符合' if coef * exp > 0 else '✗ 不符'
    print(f'  {nm:<16} {coef:>+.4f}  {ok}')
print(f'\n下一步: sudo docker cp ros1_gnss:/root/cn0_model_report.html ~/cn0_model_report.html')
