#!/usr/bin/env python3
"""
lidar_step8f_lidar_vs_elevation_ablation.py
消融实验：量化 LiDAR 特征在"控制仰角后"的增量判别力。

step8e 发现多特征 AUC=0.906，但 elevation 特征重要性=74.6%。
问题：LiDAR-特有特征（delta_L, incidence, rho_norm）是否有超出仰角的增量信号？

三组模型（5折交叉验证 AUC）：
  M1  仅 elevation
  M2  仅 LiDAR 特征（delta_L + incidence + rho_norm，无仰角）
  M3  elevation + LiDAR 特征（全特征）

增量 = M3_AUC − M1_AUC
  >0.05 → LiDAR 有实质增量贡献（研究有价值）
  0.02–0.05 → 微弱增量（可作辅助特征，难独立成主要贡献）
  <0.02 → LiDAR 特征几乎不超出仰角已知信息

输入:  --data dd_resid_material.csv  (step8d 输出)
用法:
  python3 lidar_step8f_lidar_vs_elevation_ablation.py \
    --data /root/dd_resid_material.csv --thr_hi 20 --thr_lo 5
"""

import argparse, csv
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.metrics import roc_auc_score
from sklearn.preprocessing import StandardScaler

ap = argparse.ArgumentParser()
ap.add_argument('--data',   default='/root/dd_resid_material.csv')
ap.add_argument('--thr_hi', type=float, default=20.0)
ap.add_argument('--thr_lo', type=float, default=5.0)
ap.add_argument('--seed',   type=int,   default=42)
args = ap.parse_args()

# ── 读数据 ────────────────────────────────────────────────────────────────────
rows = []
with open(args.data) as f:
    for r in csv.DictReader(f):
        try:
            dd  = float(r['dd_resid'])
            dL  = float(r['delta_L']) if r.get('delta_L') not in (None,'','nan') else np.nan
            if np.isnan(dL): continue
            rows.append({
                'dd_abs':   abs(dd),
                'delta_L':  dL,
                'incidence':float(r['incidence']) if r.get('incidence') else 45.0,
                'rho_norm': float(r['rho_norm']),
                'elev':     float(r['elev']) if r.get('elev') not in (None,'','nan') else np.nan,
            })
        except (ValueError, KeyError):
            continue

dd_abs = np.array([r['dd_abs'] for r in rows])
pos  = dd_abs > args.thr_hi
neg  = dd_abs < args.thr_lo
keep = pos | neg
y    = pos[keep].astype(int)
print(f'样本: {keep.sum()}  正类(真NLOS): {y.sum()}  负类(干净): {(y==0).sum()}')

def col(name):
    arr = np.array([r[name] for r in rows])[keep]
    if name == 'elev':
        arr = np.where(np.isnan(arr), np.nanmedian(arr), arr)
    return arr

elev    = col('elev')
dL      = col('delta_L')
inc     = col('incidence')
rho     = np.log(np.clip(col('rho_norm'), 1e-3, None))

# ── 特征组定义 ────────────────────────────────────────────────────────────────
feature_sets = {
    'M1_elevation_only':   np.column_stack([elev]),
    'M2_lidar_only':       np.column_stack([dL, inc, rho]),
    'M3_elevation+lidar':  np.column_stack([elev, dL, inc, rho]),
}

cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=args.seed)

print(f'\n{"="*65}')
print(f'{"模型":<22}  {"LogReg AUC":>12}  {"RF AUC":>10}  {"GBT AUC":>10}')
print(f'{"-"*65}')

results = {}
for mname, X in feature_sets.items():
    Xs = StandardScaler().fit_transform(X)
    lr  = LogisticRegression(max_iter=1000)
    rf  = RandomForestClassifier(n_estimators=200, max_depth=4,
                                  min_samples_leaf=15, random_state=args.seed)
    gbt = GradientBoostingClassifier(n_estimators=200, max_depth=3,
                                      min_samples_leaf=15, random_state=args.seed)
    auc_lr  = cross_val_score(lr,  Xs, y, cv=cv, scoring='roc_auc')
    auc_rf  = cross_val_score(rf,  X,  y, cv=cv, scoring='roc_auc')
    auc_gbt = cross_val_score(gbt, X,  y, cv=cv, scoring='roc_auc')
    results[mname] = {
        'lr':  auc_lr.mean(),
        'rf':  auc_rf.mean(),
        'gbt': auc_gbt.mean(),
    }
    best = max(auc_lr.mean(), auc_rf.mean(), auc_gbt.mean())
    print(f'{mname:<22}  {auc_lr.mean():.3f}±{auc_lr.std():.3f}  '
          f'{auc_rf.mean():.3f}±{auc_rf.std():.3f}  '
          f'{auc_gbt.mean():.3f}±{auc_gbt.std():.3f}')

# ── 增量计算 ─────────────────────────────────────────────────────────────────
print(f'\n{"="*65}')
print('LiDAR 增量（M3 − M1，最佳分类器）')
print(f'{"="*65}')
for clf in ['lr', 'rf', 'gbt']:
    m1 = results['M1_elevation_only'][clf]
    m2 = results['M2_lidar_only'][clf]
    m3 = results['M3_elevation+lidar'][clf]
    delta = m3 - m1
    print(f'  [{clf.upper()}]  M1={m1:.3f}  M2={m2:.3f}  M3={m3:.3f}  '
          f'增量(M3−M1)={delta:+.3f}')

# 用最佳分类器给出裁决
best_m1 = max(v['rf'] for k,v in results.items() if 'M1' in k)
best_m3 = max(v['rf'] for k,v in results.items() if 'M3' in k)
best_m2 = max(v['rf'] for k,v in results.items() if 'M2' in k)
delta_rf = best_m3 - best_m1

print(f'\n{"="*65}')
print('裁决（基于随机森林）')
print(f'{"="*65}')
print(f'  仅仰角 AUC = {best_m1:.3f}')
print(f'  仅LiDAR AUC = {best_m2:.3f}')
print(f'  仰角+LiDAR AUC = {best_m3:.3f}')
print(f'  LiDAR 增量 = {delta_rf:+.3f}')

if delta_rf > 0.05:
    print('\n  ✅ LiDAR 特征有实质增量贡献（超出仰角已知信息）')
    print('     → 研究命题成立：LiDAR 几何/材质特征提升 NLOS 检测性能')
    print('     → 建议：整理成"LiDAR-aided NLOS detection"论文')
elif delta_rf > 0.02:
    print('\n  🔶 LiDAR 特征有微弱但可测的增量贡献')
    print('     → 可作辅助特征；增量有限，需结合更多特征或更多数据')
    print('     → 建议：以"辅助特征"角色写条件性结论')
else:
    print('\n  ⚠️ LiDAR 特征几乎不超出仰角已知信息（增量<0.02）')
    print(f'     → 仰角 AUC={best_m1:.3f} 已接近全特征 AUC={best_m3:.3f}')
    print('     → step8e 的"方向B"主要是仰角在判别，非LiDAR材质/几何')
    print('     → 诚实结论：LiDAR 几何遮挡对 NLOS 无显著独立判别贡献')
    print('     → 建议：把"负/边界结论+机理分析"整理成有价值的边界论文')

# ── 附加：仰角分组内的 LiDAR 判别力 ─────────────────────────────────────────
print(f'\n{"="*65}')
print('附加：按仰角分层后，LiDAR-only 在各层的 AUC')
print('（若高/低仰角层内 LiDAR 仍有 AUC>0.6，则有局部贡献）')
print(f'{"="*65}')
elev_all = elev
lo_thr = np.percentile(elev_all, 33)
hi_thr = np.percentile(elev_all, 67)
bins = [
    ('低仰角 (<33th)', elev_all < lo_thr),
    ('中仰角 (33-67th)', (elev_all >= lo_thr) & (elev_all < hi_thr)),
    ('高仰角 (>67th)', elev_all >= hi_thr),
]
X_lidar = np.column_stack([dL, inc, rho])
for label, mask in bins:
    ym = y[mask]; Xm = X_lidar[mask]
    if ym.sum() < 10 or (ym==0).sum() < 10:
        print(f'  {label}: 样本不足（正类={ym.sum()}, 负类={(ym==0).sum()}）')
        continue
    try:
        rf_m = RandomForestClassifier(n_estimators=100, max_depth=4,
                                       min_samples_leaf=10, random_state=args.seed)
        cv_m = StratifiedKFold(n_splits=min(5, ym.sum()), shuffle=True, random_state=args.seed)
        auc_m = cross_val_score(rf_m, Xm, ym, cv=cv_m, scoring='roc_auc')
        print(f'  {label:<22}: n={mask.sum():>5}  正类={ym.sum():>4}  '
              f'LiDAR-only AUC={auc_m.mean():.3f}±{auc_m.std():.3f}')
    except Exception as e:
        print(f'  {label}: 计算失败 ({e})')
