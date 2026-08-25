#!/usr/bin/env python3
"""
lidar_step6f_multignss_rho_cn0.py
多星座（G/C/E）版的 ρ_norm vs CN0_drop 关联数据构建。

与 step6c 的区别：
  - step6c 的 CN0/仰角/LOS标签来自 del2AINLOS 的 training_data.csv（GPS-only）
  - step6f 直接用 epoch_sat_azel_multignss.csv（自带 cn0/elevation/sys，全星座，
    LOS+NLOS 都有），不依赖 del2AINLOS
  - CN0_expected 基准曲线**按星座分别拟合**（G/C/E 的 CN0 基线不同，不能混拟）

LOS/NLOS 判定：
  某 (utc_t, sat_id) 若出现在反射文件中 → NLOS；否则 → LOS（用于拟合基准）

输入:
  --azel  epoch_sat_azel_multignss.csv  (step3 输出，全星座 cn0/elevation)
  --refl  lidar_reflection_intensity_mg.csv  (step6b 在多星座反射上的输出，含 rho_norm)

输出:
  --out_csv  rho_cn0_analysis_mg.csv  (与 step6c 同格式，step6e 可直接使用)

用法:
  python3 lidar_step6f_multignss_rho_cn0.py \\
    --azel /root/epoch_sat_azel_multignss.csv \\
    --refl /root/lidar_reflection_intensity_mg.csv \\
    --out_csv /root/rho_cn0_analysis_mg.csv \\
    --cn0_poly 2
"""

import argparse, csv, math
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--azel',     default='/root/epoch_sat_azel_multignss.csv')
ap.add_argument('--refl',     default='/root/lidar_reflection_intensity_mg.csv')
ap.add_argument('--out_csv',  default='/root/rho_cn0_analysis_mg.csv')
ap.add_argument('--cn0_poly', type=int, default=2)
args = ap.parse_args()

TKEY = lambda t: round(float(t), 1)   # 时间对齐键（0.1s）

# ── 读 azel（全星座 cn0 / elevation）─────────────────────────────────────────
print('读取多星座 azel...')
azel = []
with open(args.azel) as f:
    for r in csv.DictReader(f):
        try:
            cn0 = float(r['cn0']) if r.get('cn0') not in (None, '') else 0.0
            azel.append({
                'utc_t':  float(r['utc_t']),
                'sat_id': r['sat_id'],
                'sys':    r['sys'],
                'elev':   float(r['elevation_deg']),
                'cn0':    cn0,
            })
        except (ValueError, KeyError):
            continue
from collections import Counter
print(f'  {len(azel)} 条可见卫星记录')
print(f'  星座分布: {dict(Counter(a["sys"] for a in azel))}')
print(f'  CN0>0 的记录: {sum(1 for a in azel if a["cn0"] > 0)}')

# azel 查询字典: (tkey, sat_id) → {elev, cn0, sys}
azel_dict = {(TKEY(a['utc_t']), a['sat_id']): a for a in azel}

# ── 读反射率（NLOS 反射点，含 rho_norm）──────────────────────────────────────
print('读取多星座反射率数据...')
refl = []
with open(args.refl) as f:
    for r in csv.DictReader(f):
        if r.get('rho_norm', '') == '':
            continue
        try:
            refl.append({
                'utc_t':     float(r['utc_t']),
                'sat_id':    r['sat_id'],
                'sys':       r.get('sys', r['sat_id'][0]),
                'rho_norm':  float(r['rho_norm']),
                'intensity': r.get('intensity', ''),
                'hit_dist':  float(r['hit_dist_m'])   if r.get('hit_dist_m')   else np.nan,
                'incidence': float(r['incidence_deg']) if r.get('incidence_deg') else 45.0,
                'delta_L':   r.get('delta_L_m', ''),
                'severity':  r.get('severity', ''),
                'elev':      float(r['elevation_deg']) if r.get('elevation_deg') else np.nan,
            })
        except (ValueError, KeyError):
            continue
print(f'  {len(refl)} 条有效反射率记录（rho_norm 非空）')
print(f'  星座分布: {dict(Counter(r["sys"] for r in refl))}')

# NLOS 集合（用于从 LOS 中剔除）
nlos_set = {(TKEY(r['utc_t']), r['sat_id']) for r in refl}

# ── 按星座拟合 CN0_expected(elev) 基准曲线（仅用 LOS）─────────────────────────
print('\n按星座拟合 CN0-elevation 基准（仅 LOS 卫星）:')
cn0_models = {}
for sysc in ['G', 'C', 'E']:
    los = [a for a in azel
           if a['sys'] == sysc and a['cn0'] > 5
           and (TKEY(a['utc_t']), a['sat_id']) not in nlos_set]
    if len(los) < 20:
        print(f'  [{sysc}] LOS 样本不足 ({len(los)})，跳过')
        continue
    elev = np.array([a['elev'] for a in los])
    cn0  = np.array([a['cn0']  for a in los])
    coeffs = np.polyfit(elev, cn0, args.cn0_poly)
    cn0_models[sysc] = np.poly1d(coeffs)
    print(f'  [{sysc}] LOS={len(los):4d}  '
          f'CN0(10°)={cn0_models[sysc](10):.1f}  '
          f'CN0(30°)={cn0_models[sysc](30):.1f}  '
          f'CN0(60°)={cn0_models[sysc](60):.1f} dB-Hz')

# ── 为每个 NLOS 反射点计算 CN0_drop ──────────────────────────────────────────
print('\n匹配 CN0 并计算 CN0_drop...')
merged = []
n_no_azel = 0
n_no_model = 0
for r in refl:
    key = (TKEY(r['utc_t']), r['sat_id'])
    a = azel_dict.get(key)
    if a is None or a['cn0'] <= 0:
        n_no_azel += 1
        continue
    model = cn0_models.get(r['sys'])
    if model is None:
        n_no_model += 1
        continue
    elev = a['elev']
    cn0  = a['cn0']
    cn0_exp = float(model(elev))
    merged.append({**r,
                   'cn0': cn0,
                   'cn0_expected': cn0_exp,
                   'cn0_drop': cn0_exp - cn0,
                   'elev_used': elev})

print(f'  匹配成功: {len(merged)} / {len(refl)} '
      f'({100*len(merged)/max(len(refl),1):.1f}%)')
print(f'  未匹配到 azel/CN0: {n_no_azel}，无该星座基准: {n_no_model}')

if not merged:
    print('错误：无匹配记录')
    raise SystemExit(1)

# ── 快速分星座统计 ───────────────────────────────────────────────────────────
print('\n按星座 CN0_drop 概况:')
for sysc in ['G', 'C', 'E']:
    sub = [m for m in merged if m['sys'] == sysc]
    if not sub:
        continue
    drops = np.array([m['cn0_drop'] for m in sub])
    rhos  = np.array([m['rho_norm'] for m in sub])
    print(f'  [{sysc}] n={len(sub):4d}  '
          f'CN0_drop均值={drops.mean():+.2f}dB  '
          f'ρ_norm中位={np.median(rhos):.2f}')

# ── 写出（与 step6c 同格式 + 额外 sys 列）─────────────────────────────────────
print(f'\n写出: {args.out_csv}')
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=[
        'epoch','sat_id','sys','severity','rho_norm','intensity',
        'hit_dist_m','incidence_deg','delta_L_m',
        'cn0','cn0_expected','cn0_drop','elevation','nlos_label'])
    w.writeheader()
    for m in merged:
        w.writerow({
            'epoch':         f"{m['utc_t']:.3f}",
            'sat_id':        m['sat_id'],
            'sys':           m['sys'],
            'severity':      m['severity'],
            'rho_norm':      f"{m['rho_norm']:.4f}",
            'intensity':     m['intensity'],
            'hit_dist_m':    f"{m['hit_dist']:.2f}" if not (isinstance(m['hit_dist'], float) and math.isnan(m['hit_dist'])) else '',
            'incidence_deg': f"{m['incidence']:.1f}",
            'delta_L_m':     m['delta_L'],
            'cn0':           f"{m['cn0']:.1f}",
            'cn0_expected':  f"{m['cn0_expected']:.2f}",
            'cn0_drop':      f"{m['cn0_drop']:.3f}",
            'elevation':     f"{m['elev_used']:.1f}",
            'nlos_label':    1,
        })

print(f'完成: {len(merged)} 行')
print(f'\n下一步（在扩大样本上重新验证材质效应）:')
print(f'  python3 scripts/lidar_step6e_verify_material.py \\')
print(f'    --data {args.out_csv} \\')
print(f'    --out_csv  /root/material_verify_mg.csv \\')
print(f'    --out_html /root/material_verify_mg.html')
