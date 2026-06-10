#!/usr/bin/env python3
"""
lidar_step6d_cn0_model.py  (v2 — 改进版 CN0_expected 模型)
多变量 CN0_drop 预测模型（物理驱动线性回归 + 随机森林）

物理基础：
    CN0_drop ≈ α₁·log(R) − α₂·log(ρ) + α₃·log(1/cos θ) + f(severity)

CN0_expected 改进（v2 新增）：
    原版: 全局 2 次多项式 elev → 残留仰角相关性（RF 仰角重要性 56%）
    改进: 全局 sin(elev) 线性基准 + 逐颗卫星偏差校正
        CN0_exp(sat, elev) = a·sin(elev) + b + δ_sat
        δ_sat = mean(CN0_LOS_sat - 全局基准)，仅用 ≥5 个 LOS 历元的卫星
    物理依据：大气损耗 ∝ 1/sin(elev)；GPS 星座各星发射功率差 ~1-3 dB

时序划分：按 epoch 排序，前 70% 训练，后 30% 测试

输入:
  --data  rho_cn0_analysis.csv   (step6c 输出，含 cn0/elevation/sat_id)
  --gnss  training_data.csv      (del2AINLOS 输出，用于重新拟合 CN0_expected)

输出:
  --out_csv   cn0_model_results.csv
  --out_html  cn0_model_report.html

用法:
  python3 lidar_step6d_cn0_model.py \\
    --data /root/rho_cn0_analysis.csv \\
    --gnss /root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS/data/UrbanNavMedium/training_data.csv \\
    --out_csv  /root/cn0_model_results.csv \\
    --out_html /root/cn0_model_report.html
"""

import argparse, csv, math, json, warnings
from collections import defaultdict
import numpy as np
from sklearn.linear_model import Ridge
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
import scipy.stats as sps

warnings.filterwarnings('ignore')

ap = argparse.ArgumentParser()
ap.add_argument('--data',      default='/root/rho_cn0_analysis.csv')
ap.add_argument('--gnss',      default=(
    '/root/gnss_ws/src/PSRI-73-2309-PR-Dev-main/rospak/src/del2AINLOS'
    '/data/UrbanNavMedium/training_data.csv'))
ap.add_argument('--out_csv',   default='/root/cn0_model_results.csv')
ap.add_argument('--out_html',  default='/root/cn0_model_report.html')
ap.add_argument('--test_frac', type=float, default=0.3)
args = ap.parse_args()

# ── 1. 拟合改进版 CN0_expected（全局 sin + 逐星偏差）─────────────────────────
print('拟合改进版 CN0_expected...')
gnss_rows = []
with open(args.gnss) as f:
    for r in csv.DictReader(f):
        try:
            gnss_rows.append({
                'epoch':      float(r['epoch']),
                'sat_id':     r['sat_id'],
                'cn0':        float(r['cn0']),
                'elevation':  float(r['elevation']),
                'nlos_label': int(r['nlos_label']),
            })
        except (ValueError, KeyError):
            pass

los = [r for r in gnss_rows if r['nlos_label'] == 0 and r['cn0'] > 5]
print(f'  LOS 样本: {len(los)}, 卫星数: {len(set(r["sat_id"] for r in los))}')

# 全局基准：CN0 = a·sin(elev) + b  (最小二乘)
sin_e = np.array([math.sin(math.radians(r['elevation'])) for r in los])
cn0_a = np.array([r['cn0'] for r in los])
A = np.column_stack([sin_e, np.ones(len(sin_e))])
(a_g, b_g), *_ = np.linalg.lstsq(A, cn0_a, rcond=None)
print(f'  全局基准: CN0 = {a_g:.3f}·sin(elev) + {b_g:.3f}')
print(f'    10°: {a_g*math.sin(math.radians(10))+b_g:.1f}  '
      f'30°: {a_g*math.sin(math.radians(30))+b_g:.1f}  '
      f'60°: {a_g*math.sin(math.radians(60))+b_g:.1f} dB-Hz')

# 逐颗卫星偏差
sat_res = defaultdict(list)
for r in los:
    sat_res[r['sat_id']].append(
        r['cn0'] - (a_g * math.sin(math.radians(r['elevation'])) + b_g))
sat_bias = {s: float(np.mean(v)) for s, v in sat_res.items() if len(v) >= 5}
print(f'  卫星偏差: {len(sat_bias)} 颗，'
      f'范围 [{min(sat_bias.values()):.2f}, {max(sat_bias.values()):.2f}] dB')
for s, b in sorted(sat_bias.items()):
    print(f'    {s}: {b:+.2f} dB  (n={len(sat_res[s])})')

def cn0_exp_new(sat_id, elev_deg):
    return a_g * math.sin(math.radians(elev_deg)) + b_g + sat_bias.get(sat_id, 0.0)

# ── 2. 读反射率+CN0 数据，重算 CN0_drop ───────────────────────────────────────
print('\n读取 rho_cn0_analysis.csv...')
rows = []
with open(args.data) as f:
    for r in csv.DictReader(f):
        try:
            rho  = float(r['rho_norm'])
            dist = float(r['hit_dist_m'])
            inc  = float(r['incidence_deg']) if r.get('incidence_deg') else 45.0
            elev = float(r['elevation'])
            cn0  = float(r['cn0'])
            sev  = r.get('severity', 'strong')
            sat  = r.get('sat_id', '')
            epch = float(r['epoch'])
            drop_old = float(r['cn0_drop']) if r.get('cn0_drop') else 0.0
            if rho <= 0 or dist <= 0:
                continue
            drop_new = cn0_exp_new(sat, elev) - cn0
            rows.append({
                'epoch': epch, 'sat_id': sat,
                'rho_norm': rho, 'hit_dist_m': dist,
                'incidence_deg': inc, 'elevation': elev,
                'cn0': cn0,
                'cn0_drop_old': drop_old,
                'cn0_drop': drop_new,   # 用改进版
                'severity': sev,
            })
        except (ValueError, KeyError):
            pass

n = len(rows)
print(f'  有效样本: {n}')
for s in ['mild', 'strong', 'severe']:
    print(f'    {s}: {sum(1 for r in rows if r["severity"]==s)}')

# ── 3. 诊断：改进前后 elevation 残差相关性 ────────────────────────────────────
elev_arr  = np.array([r['elevation']    for r in rows])
drop_old  = np.array([r['cn0_drop_old'] for r in rows])
drop_new  = np.array([r['cn0_drop']     for r in rows])

r_old, _ = sps.pearsonr(elev_arr, drop_old)
r_new, _ = sps.pearsonr(elev_arr, drop_new)
print(f'\nCN0_drop vs elevation 相关性（期望趋近 0）:')
print(f'  旧版(poly2)  r = {r_old:+.4f}')
print(f'  新版(per_sat) r = {r_new:+.4f}  '
      f'{"↓ 改善 {:.0f}%".format(100*(abs(r_old)-abs(r_new))/abs(r_old)) if abs(r_old)>1e-6 else ""}')
print(f'\nCN0_drop 统计:')
for label, arr in [('旧版', drop_old), ('新版', drop_new)]:
    print(f'  {label}: 均值={arr.mean():.3f}  std={arr.std():.3f}  '
          f'5th={np.percentile(arr,5):.2f}  95th={np.percentile(arr,95):.2f}')

# ── 4. 特征工程 ───────────────────────────────────────────────────────────────
SEV_CODE        = {'mild': 0, 'strong': 1, 'severe': 2}
PHYS_FEAT_NAMES = ['log(ρ_norm)', 'log(R_hit)', '-log(cos α)', 'sin(elev)', 'severity']
RF_FEAT_NAMES   = ['ρ_norm', 'R(m)', 'α(°)', 'elev(°)', 'severity',
                   'log(ρ)', 'log(R)', '-log(cosα)']
EXPECTED_SIGNS  = [-1, +1, +1, -1, +1]

def make_features(batch):
    Xp, Xr, y = [], [], []
    for r in batch:
        rho = r['rho_norm']; dist = r['hit_dist_m']
        inc = r['incidence_deg']; elev = r['elevation']
        sev = SEV_CODE.get(r['severity'], 1)
        cos_inc    = max(math.cos(math.radians(inc)), 0.05)
        log_rho    =  math.log(rho)
        log_dist   =  math.log(dist)
        neg_lc     = -math.log(cos_inc)
        sin_elev   =  math.sin(math.radians(elev))
        Xp.append([log_rho, log_dist, neg_lc, sin_elev, sev])
        Xr.append([rho, dist, inc, elev, sev, log_rho, log_dist, neg_lc])
        y.append(r['cn0_drop'])
    return np.array(Xp), np.array(Xr), np.array(y)

# ── 5. 时序划分 ───────────────────────────────────────────────────────────────
rows_s  = sorted(rows, key=lambda r: r['epoch'])
n_te    = int(n * args.test_frac)
n_tr    = n - n_te
tr_rows = rows_s[:n_tr];  te_rows = rows_s[n_tr:]

print(f'\n时序划分: 训练 {n_tr}, 测试 {n_te}')
print(f'  训练 epoch: [{tr_rows[0]["epoch"]:.0f}, {tr_rows[-1]["epoch"]:.0f}]')
print(f'  测试 epoch: [{te_rows[0]["epoch"]:.0f}, {te_rows[-1]["epoch"]:.0f}]')

Xp_tr, Xr_tr, y_tr = make_features(tr_rows)
Xp_te, Xr_te, y_te = make_features(te_rows)

# ── 6. 模型1: 物理线性回归 ────────────────────────────────────────────────────
print('\n── 模型1: 物理线性回归 (Ridge) ──')
lr = Ridge(alpha=0.1)
lr.fit(Xp_tr, y_tr)
yp_lr_tr = lr.predict(Xp_tr);  yp_lr_te = lr.predict(Xp_te)
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

# ── 7. 模型2: 随机森林（更保守的超参数减少过拟合）─────────────────────────────
print('\n── 模型2: 随机森林 ──')
rf = RandomForestRegressor(n_estimators=200, max_depth=5, min_samples_leaf=20,
                            max_features=0.6, random_state=42, n_jobs=-1)
rf.fit(Xr_tr, y_tr)
yp_rf_tr = rf.predict(Xr_tr);  yp_rf_te = rf.predict(Xr_te)
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

# ── 8. Baseline + 对比 ────────────────────────────────────────────────────────
sev_mean    = {s: float(np.mean([r['cn0_drop'] for r in tr_rows if r['severity']==s]))
               for s in ['mild','strong','severe']}
global_mean = float(np.mean(y_tr))
yp_base_te  = np.array([sev_mean.get(r['severity'], global_mean) for r in te_rows])
r2_base     = r2_score(y_te, yp_base_te)
mae_base    = mean_absolute_error(y_te, yp_base_te)

print(f'\n── 总结 ──')
print(f'  {"模型":<18} {"测试R²":>8}  {"MAE":>8}  {"vs Baseline":>12}')
print(f'  {"Baseline":18} {r2_base:>8.4f}  {mae_base:>8.3f}')
print(f'  {"PhysLR":18} {r2_lr_te:>8.4f}  {mae_lr:>8.3f}  {r2_lr_te-r2_base:>+12.4f}')
print(f'  {"RF":18} {r2_rf_te:>8.4f}  {mae_rf:>8.3f}  {r2_rf_te-r2_base:>+12.4f}')

# ── 9. 按 severity 分组（测试集）─────────────────────────────────────────────
print(f'\n── 按 severity（测试集）──')
sev_te = np.array([r['severity'] for r in te_rows])
print(f'  {"sev":<8} {"n":>5}  {"LR_R²":>8}  {"RF_R²":>8}  {"LR_MAE":>8}')
for s in ['mild', 'strong', 'severe']:
    m = sev_te == s
    if m.sum() < 5: continue
    print(f'  {s:<8} {m.sum():>5}  '
          f'{r2_score(y_te[m],yp_lr_te[m]):>8.4f}  '
          f'{r2_score(y_te[m],yp_rf_te[m]):>8.4f}  '
          f'{mean_absolute_error(y_te[m],yp_lr_te[m]):>8.3f}')

# ── 10. 残差分析 ──────────────────────────────────────────────────────────────
resid = y_te - yp_lr_te
_, p_norm = sps.normaltest(resid)
r_resid_elev, _ = sps.pearsonr(np.array([r['elevation'] for r in te_rows]), resid)
print(f'\n── 线性模型残差（测试集）──')
print(f'  均值={resid.mean():.3f}  std={resid.std():.3f}  '
      f'5th={np.percentile(resid,5):.2f}  95th={np.percentile(resid,95):.2f} dB')
print(f'  正态检验 p={p_norm:.3e}')
print(f'  残差 vs elevation 相关性: r={r_resid_elev:.4f}  '
      f'(接近0则 elevation 残差已清除)')

# ── 11. 写 CSV ────────────────────────────────────────────────────────────────
print(f'\n写出: {args.out_csv}')
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=[
        'epoch','sat_id','severity','split',
        'rho_norm','hit_dist_m','incidence_deg','elevation',
        'cn0','cn0_drop','pred_lr','pred_rf','resid_lr','resid_rf'])
    w.writeheader()
    for batch, yp_lr, yp_rf, spl in [
            (tr_rows, yp_lr_tr, yp_rf_tr, 'train'),
            (te_rows, yp_lr_te, yp_rf_te, 'test')]:
        for r, pl, pf in zip(batch, yp_lr.tolist(), yp_rf.tolist()):
            w.writerow({
                'epoch': f"{r['epoch']:.3f}", 'sat_id': r['sat_id'],
                'severity': r['severity'], 'split': spl,
                'rho_norm': f"{r['rho_norm']:.4f}",
                'hit_dist_m': f"{r['hit_dist_m']:.2f}",
                'incidence_deg': f"{r['incidence_deg']:.1f}",
                'elevation': f"{r['elevation']:.1f}",
                'cn0': f"{r['cn0']:.1f}",
                'cn0_drop': f"{r['cn0_drop']:.3f}",
                'pred_lr': f"{pl:.3f}", 'pred_rf': f"{pf:.3f}",
                'resid_lr': f"{r['cn0_drop']-pl:.3f}",
                'resid_rf': f"{r['cn0_drop']-pf:.3f}",
            })

# ── 12. HTML 报告 ─────────────────────────────────────────────────────────────
print(f'写出: {args.out_html}')

SEV_COL = {'mild': '#42A5F5', 'strong': '#FFA726', 'severe': '#EF5350'}

page_data = {
    'summary': {
        'n_train': n_tr, 'n_test': n_te,
        'correction': {
            'r_elev_old': round(float(r_old), 4),
            'r_elev_new': round(float(r_new), 4),
            'std_old': round(float(drop_old.std()), 3),
            'std_new': round(float(drop_new.std()), 3),
        },
        'models': [
            {'name': 'Baseline (severity均值)', 'r2_te': round(r2_base,4),
             'r2_tr': None, 'mae': round(mae_base,3), 'rmse': None, 'delta': 0},
            {'name': 'PhysLR (改进CN0_exp)', 'r2_te': round(r2_lr_te,4),
             'r2_tr': round(r2_lr_tr,4), 'mae': round(mae_lr,3),
             'rmse': round(rmse_lr,3), 'delta': round(r2_lr_te-r2_base,4)},
            {'name': 'RF (改进CN0_exp)', 'r2_te': round(r2_rf_te,4),
             'r2_tr': round(r2_rf_tr,4), 'mae': round(mae_rf,3),
             'rmse': round(rmse_rf,3), 'delta': round(r2_rf_te-r2_base,4)},
        ]
    },
    'coefs': [
        {'name': nm, 'coef': round(float(c),4),
         'expected': int(e), 'ok': bool(c * e > 0)}
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
    'resid': [round(float(v),3) for v in resid.tolist()],
    'sat_bias': sorted(
        [{'sat': s, 'bias': round(b,3)} for s, b in sat_bias.items()],
        key=lambda x: x['bias']),
    'correction_scatter': [
        {'elev': round(float(elev_arr[i]),1),
         'drop_old': round(float(drop_old[i]),2),
         'drop_new': round(float(drop_new[i]),2)}
        for i in range(0, len(rows), max(1, len(rows)//600))
    ],
}

html = """<!DOCTYPE html>
<html lang="zh"><head>
<meta charset="UTF-8">
<title>CN0_drop 预测模型报告 v2</title>
<style>
  body{font-family:Arial,sans-serif;max-width:1150px;margin:20px auto;background:#f4f5f7;color:#333}
  h1{color:#1565C0;border-bottom:3px solid #1565C0;padding-bottom:8px}
  h2{color:#283593;margin-top:30px}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:16px}
  .card{background:#fff;border-radius:8px;padding:18px;box-shadow:0 1px 5px rgba(0,0,0,.1)}
  table{border-collapse:collapse;width:100%}
  th{background:#1565C0;color:#fff;padding:7px 11px;text-align:left}
  td{padding:6px 10px;border-bottom:1px solid #eee}
  tr:hover td{background:#f0f4ff}
  .ok{color:#2e7d32;font-weight:bold} .bad{color:#c62828;font-weight:bold}
  .tag{display:inline-block;padding:2px 8px;border-radius:10px;font-size:.82em}
  .tag-blue{background:#e3f2fd;color:#1565C0}
  .tag-green{background:#e8f5e9;color:#2e7d32}
  canvas{display:block;margin:0 auto}
  .note{color:#666;font-size:.88em;margin-top:6px}
  .badge{display:inline-block;padding:3px 9px;border-radius:4px;font-weight:bold;font-size:.9em}
  .badge-up{background:#e8f5e9;color:#2e7d32}
  .badge-dn{background:#ffebee;color:#c62828}
</style>
</head><body>
<h1>CN0_drop 预测模型报告 <span class="tag tag-blue">v2 改进版</span></h1>
<p>改进: <b>全局 2 次多项式 → sin(elev) 线性 + 逐颗卫星偏差校正</b>，降低 CN0_drop 中的仰角残差</p>

<h2>1. CN0_expected 改进效果</h2>
<div class="grid2">
<div class="card">
  <b>改进前后对比</b>
  <table style="margin-top:10px">
  <tr><th>指标</th><th>旧版 (poly2)</th><th>新版 (per_sat)</th><th>变化</th></tr>
  <tr id="tr_r_elev"></tr>
  <tr id="tr_std"></tr>
  </table>
  <p class="note">elevation vs CN0_drop 相关性越接近 0，说明仰角依赖去除越彻底</p>
</div>
<div class="card">
  <b>逐颗卫星偏差 (δ_sat)</b>
  <canvas id="bias_canvas" width="420" height="220"></canvas>
  <p class="note">各 GPS 卫星发射功率差异导致的系统性偏差</p>
</div>
</div>

<h2>2. 模型性能对比（测试集，时序划分）</h2>
<div class="card">
<table><tr><th>模型</th><th>测试 R²</th><th>训练 R²</th><th>MAE (dB)</th><th>RMSE (dB)</th><th>ΔR² vs Baseline</th></tr>
<tbody id="model_tbody"></tbody></table>
<p class="note">时序划分: 前 70% 训练，后 30% 测试。训练/测试 R² 差距大 = 过拟合。</p>
</div>

<h2>3. 物理系数验证</h2>
<div class="card">
<p>理论: CN0_drop ≈ α₁·log(R) − α₂·log(ρ) + α₃·log(1/cosθ) + ...</p>
<table><tr><th>特征</th><th>拟合系数</th><th>理论符号</th><th>符合?</th><th>物理解释</th></tr>
<tbody id="coef_tbody"></tbody></table>
<p class="note">截距: <span id="intercept_span"></span> dB-Hz</p>
</div>

<h2>4. 随机森林特征重要性</h2>
<div class="card">
<canvas id="fi_canvas" width="700" height="300"></canvas>
<p class="note">改进后 elevation 重要性是否下降？</p>
</div>

<h2>5. 实测 vs 预测（测试集）</h2>
<div class="grid2">
<div class="card"><b>PhysLR</b><canvas id="sc_lr" width="420" height="370"></canvas></div>
<div class="card"><b>随机森林</b><canvas id="sc_rf" width="420" height="370"></canvas></div>
</div>

<h2>6. 残差直方图（PhysLR，测试集）</h2>
<div class="card">
<canvas id="resid_cv" width="700" height="250"></canvas>
<p class="note">残差均值越接近 0，模型越无偏；std 越小越好</p>
</div>

<script>
const D = __PAGE_DATA__;

// ── 1. 改进效果表 ──
const cor = D.summary.correction;
const fmtDelta = function(a, b, better_smaller){
  const delta = b - a;
  const improved = better_smaller ? delta < 0 : delta > 0;
  return '<span class="badge '+(improved?'badge-up':'badge-dn')+'">'
    +(delta>=0?'+':'')+delta.toFixed(4)+'</span>';
};
document.getElementById('tr_r_elev').innerHTML =
  '<td>elevation vs CN0_drop 相关性 r</td>'
  +'<td>'+cor.r_elev_old+'</td><td>'+cor.r_elev_new+'</td>'
  +'<td>'+fmtDelta(Math.abs(cor.r_elev_old), Math.abs(cor.r_elev_new), true)+'</td>';
document.getElementById('tr_std').innerHTML =
  '<td>CN0_drop 标准差 (dB)</td>'
  +'<td>'+cor.std_old+'</td><td>'+cor.std_new+'</td>'
  +'<td>'+fmtDelta(cor.std_old, cor.std_new, true)+'</td>';

// ── 2. 模型性能表 ──
const mtbody = document.getElementById('model_tbody');
D.summary.models.forEach(function(m){
  const tr = document.createElement('tr');
  const cls = m.delta > 0.005 ? 'ok' : (m.delta < -0.005 ? 'bad' : '');
  tr.innerHTML = '<td>'+m.name+'</td>'
    +'<td class="'+cls+'">'+m.r2_te+'</td>'
    +'<td>'+(m.r2_tr !== null ? m.r2_tr : '—')+'</td>'
    +'<td>'+m.mae+'</td>'
    +'<td>'+(m.rmse !== null ? m.rmse : '—')+'</td>'
    +'<td>'+(m.delta !== 0 ? (m.delta>0?'<span class="badge badge-up">+':'<span class="badge badge-dn">')+m.delta+'</span>' : '—')+'</td>';
  mtbody.appendChild(tr);
});

// ── 卫星偏差柱图 ──
(function(){
  const cv = document.getElementById('bias_canvas');
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height, p = 50;
  const data = D.sat_bias;
  if(!data || data.length === 0){ ctx.fillText('无数据',W/2,H/2); return; }
  const biases = data.map(function(d){return d.bias;});
  const bMin = Math.min(...biases)-0.5, bMax = Math.max(...biases)+0.5;
  const barW = (W-2*p)/data.length;
  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  // zero line
  const y0 = p + (bMax/(bMax-bMin))*(H-2*p);
  ctx.strokeStyle='#999'; ctx.lineWidth=1; ctx.setLineDash([3,3]);
  ctx.beginPath(); ctx.moveTo(p,y0); ctx.lineTo(W-p,y0); ctx.stroke();
  ctx.setLineDash([]);
  data.forEach(function(d,i){
    const x   = p + i*barW + barW*0.1;
    const bw2 = barW*0.8;
    const yTop = p + (bMax-d.bias)/(bMax-bMin)*(H-2*p);
    const h    = Math.abs(yTop - y0);
    ctx.fillStyle = d.bias >= 0 ? '#42A5F5' : '#EF5350';
    ctx.fillRect(x, Math.min(yTop,y0), bw2, h);
    ctx.fillStyle='#333'; ctx.font='9px Arial'; ctx.textAlign='center';
    ctx.fillText(d.sat, x+bw2/2, H-4);
  });
  ctx.fillStyle='#555'; ctx.font='11px Arial'; ctx.textAlign='center';
  ctx.fillText('蓝=高于平均  红=低于平均', W/2, 14);
})();

// ── 3. 系数表 ──
const PHYS_EXP = {
  'log(ρ_norm)': '高反射率 → 衰减减小',
  'log(R_hit)':  '路径越远 → 衰减增大（v2反号=近场遮挡主导）',
  '-log(cos α)': '掠射角越大 → 衰减增大',
  'sin(elev)':   '高仰角 → 基准CN0更高',
  'severity':    'mild<strong<severe 路径差增大',
};
document.getElementById('intercept_span').textContent = D.intercept;
const ctbody = document.getElementById('coef_tbody');
D.coefs.forEach(function(c){
  const tr = document.createElement('tr');
  tr.innerHTML = '<td>'+c.name+'</td>'
    +'<td style="font-weight:bold">'+(c.coef>=0?'+':'')+c.coef+'</td>'
    +'<td>'+(c.expected>0?'+':'−')+'</td>'
    +'<td class="'+(c.ok?'ok':'bad')+'">'+(c.ok?'✓':'✗')+'</td>'
    +'<td>'+(PHYS_EXP[c.name]||'')+'</td>';
  ctbody.appendChild(tr);
});

// ── 4. 特征重要性 ──
(function(){
  const cv = document.getElementById('fi_canvas');
  const ctx = cv.getContext('2d');
  const W=cv.width, H=cv.height, p=50, barH=28, gap=5;
  const data = D.importances;
  const maxI = data[0].imp;
  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  const cols=['#1E88E5','#42A5F5','#64B5F6','#90CAF9','#BBDEFB','#FFA726','#FFB74D','#FFCC80'];
  data.forEach(function(d,i){
    const y  = p + i*(barH+gap);
    const bw = (W-p-140)*d.imp/maxI;
    ctx.fillStyle = cols[i%cols.length];
    ctx.fillRect(p, y, bw, barH);
    ctx.fillStyle='#333'; ctx.font='12px Arial'; ctx.textAlign='right';
    ctx.fillText(d.name, p-6, y+barH/2+4);
    ctx.textAlign='left';
    ctx.fillText((d.imp*100).toFixed(1)+'%', p+bw+4, y+barH/2+4);
  });
  ctx.fillStyle='#333'; ctx.font='bold 13px Arial'; ctx.textAlign='center';
  ctx.fillText('随机森林特征重要性', W/2, H-6);
})();

// ── 5. 散点图 ──
function drawScatter(cid, key, r2val){
  const cv = document.getElementById(cid);
  const ctx = cv.getContext('2d');
  const W=cv.width, H=cv.height, p=46;
  const data = D.scatter;
  const vals = [].concat(data.map(function(d){return d.actual;}),
                          data.map(function(d){return d[key];}));
  const vMin = Math.floor(Math.min.apply(null,vals))-1;
  const vMax = Math.ceil(Math.max.apply(null,vals))+1;
  const sx = function(v){return p+(v-vMin)/(vMax-vMin)*(W-2*p);};
  const sy = function(v){return H-p-(v-vMin)/(vMax-vMin)*(H-2*p);};
  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  ctx.strokeStyle='#eee'; ctx.lineWidth=0.7;
  for(let v=Math.ceil(vMin);v<=vMax;v+=2){
    ctx.beginPath(); ctx.moveTo(sx(v),p); ctx.lineTo(sx(v),H-p); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p,sy(v)); ctx.lineTo(W-p,sy(v)); ctx.stroke();
  }
  ctx.strokeStyle='#aaa'; ctx.lineWidth=1.2; ctx.setLineDash([5,4]);
  ctx.beginPath(); ctx.moveTo(sx(vMin),sy(vMin)); ctx.lineTo(sx(vMax),sy(vMax)); ctx.stroke();
  ctx.setLineDash([]);
  const SC={'mild':'#42A5F5','strong':'#FFA726','severe':'#EF5350'};
  data.forEach(function(d){
    ctx.beginPath(); ctx.arc(sx(d.actual),sy(d[key]),3,0,2*Math.PI);
    ctx.fillStyle=(SC[d.sev]||'#888')+'99'; ctx.fill();
  });
  ctx.strokeStyle='#333'; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(p,p); ctx.lineTo(p,H-p); ctx.lineTo(W-p,H-p); ctx.stroke();
  ctx.fillStyle='#333'; ctx.font='11px Arial'; ctx.textAlign='center';
  ctx.fillText('实测 CN0_drop (dB)', W/2, H-4);
  ctx.save(); ctx.translate(12,H/2); ctx.rotate(-Math.PI/2);
  ctx.fillText('预测 CN0_drop (dB)', 0, 0); ctx.restore();
  ctx.font='bold 14px Arial'; ctx.textAlign='right';
  ctx.fillText('R²='+r2val, W-p-4, p+18);
  const sl = ['mild','strong','severe'];
  sl.forEach(function(sv,i){
    const lx = p+8+i*88;
    ctx.beginPath(); ctx.arc(lx+6,H-p-14,5,0,2*Math.PI);
    ctx.fillStyle=SC[sv]; ctx.fill();
    ctx.fillStyle='#555'; ctx.font='10px Arial'; ctx.textAlign='left';
    ctx.fillText(sv, lx+14, H-p-10);
  });
}
drawScatter('sc_lr','pred_lr',D.summary.models[1].r2_te);
drawScatter('sc_rf','pred_rf',D.summary.models[2].r2_te);

// ── 6. 残差直方图 ──
(function(){
  const cv = document.getElementById('resid_cv');
  const ctx = cv.getContext('2d');
  const W=cv.width, H=cv.height, p=45;
  const resid = D.resid;
  const rMin=Math.floor(Math.min.apply(null,resid))-1;
  const rMax=Math.ceil(Math.max.apply(null,resid))+1;
  const nB=30, bv=(rMax-rMin)/nB;
  const bins=new Array(nB).fill(0);
  resid.forEach(function(v){
    const bi=Math.min(Math.floor((v-rMin)/bv),nB-1); bins[bi]++;
  });
  const mxC=Math.max.apply(null,bins);
  const bw=(W-2*p)/nB;
  ctx.fillStyle='#fafafa'; ctx.fillRect(0,0,W,H);
  bins.forEach(function(cnt,i){
    const x=p+i*bw, bh=(H-2*p)*cnt/mxC;
    ctx.fillStyle='#1E88E5cc';
    ctx.fillRect(x, H-p-bh, bw-1, bh);
  });
  const x0=p+(-rMin)/bv*bw;
  ctx.strokeStyle='#c62828'; ctx.lineWidth=1.5; ctx.setLineDash([4,3]);
  ctx.beginPath(); ctx.moveTo(x0,p); ctx.lineTo(x0,H-p); ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle='#333'; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(p,p); ctx.lineTo(p,H-p); ctx.lineTo(W-p,H-p); ctx.stroke();
  ctx.fillStyle='#333'; ctx.font='11px Arial'; ctx.textAlign='center';
  ctx.fillText('残差 (dB)', W/2, H-4);
  ctx.fillText('0', x0, H-p+13);
  const mean=resid.reduce(function(a,b){return a+b;},0)/resid.length;
  const std=Math.sqrt(resid.reduce(function(a,b){return a+b*b;},0)/resid.length - mean*mean);
  ctx.font='12px Arial'; ctx.textAlign='left'; ctx.fillStyle='#1565C0';
  ctx.fillText('均值='+mean.toFixed(3)+' dB   std='+std.toFixed(3)+' dB', p+6, p+18);
})();
</script>
</body></html>"""

html = html.replace('__PAGE_DATA__', json.dumps(page_data))
with open(args.out_html, 'w') as f:
    f.write(html)

print('\n=== 完成 ===')
print(f'  CSV:  {args.out_csv}')
print(f'  HTML: {args.out_html}')
print(f'\n拷出 HTML:')
print(f'  sudo docker cp ros1_gnss:/root/cn0_model_report.html ~/cn0_model_report.html')
