#!/usr/bin/env python3
"""
lidar_step8e_lidar_discriminates_ddnlos.py
决定性诊断：用 DD 实测 NLOS（|dd_resid| 大）当真值，测 LiDAR 特征能否判别。

背景：step8d 发现 LiDAR 标记的 NLOS 里 83% 实测无伪距偏差（dd_resid≈0）。
问题是——
  A 固有局限：LiDAR 几何遮挡 ≠ 实测 NLOS，特征也没判别力 (AUC≈0.5)
  B 可修方法：LiDAR 特征其实带信号，只是硬阈值光投不是最优 (AUC>0.65)

测试（仅在 LiDAR-NLOS 卫星内）：
  标签 y=1 若 |dd_resid|>thr_hi（实测真 NLOS）；y=0 若 |dd_resid|<thr_lo（实测干净）
  特征：delta_L, incidence, rho_norm, elevation
  报告每个特征单独 AUC + 多特征 5折交叉验证 AUC（逻辑回归 & 随机森林）

这同时解决了旧 ML 分类器 F1=0.985 的数据泄漏——现在用 DD 实测真值，诚实。

输入:  --data dd_resid_material.csv  (step8d 输出)
用法:
  python3 lidar_step8e_lidar_discriminates_ddnlos.py \\
    --data /root/dd_resid_material.csv --thr_hi 20 --thr_lo 5
"""

import argparse, csv
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.metrics import roc_auc_score
from sklearn.preprocessing import StandardScaler

ap = argparse.ArgumentParser()
ap.add_argument('--data',   default='/root/dd_resid_material.csv')
ap.add_argument('--thr_hi', type=float, default=20.0, help='|dd_resid|>此值 = 实测真NLOS')
ap.add_argument('--thr_lo', type=float, default=5.0,  help='|dd_resid|<此值 = 实测干净LOS-like')
ap.add_argument('--seed',   type=int, default=42)
args = ap.parse_args()

# ── 读数据 ────────────────────────────────────────────────────────────────────
rows = []
with open(args.data) as f:
    for r in csv.DictReader(f):
        try:
            dd = float(r['dd_resid'])
            dL = float(r['delta_L']) if r.get('delta_L') not in (None,'','nan') else np.nan
            if np.isnan(dL):
                continue
            rows.append({
                'sys':      r.get('sys',''),
                'dd_abs':   abs(dd),
                'delta_L':  dL,
                'incidence':float(r['incidence']) if r.get('incidence') else 45.0,
                'rho_norm': float(r['rho_norm']),
                'elev':     float(r['elev']) if r.get('elev') not in (None,'','nan') else np.nan,
            })
        except (ValueError, KeyError):
            continue
print(f'LiDAR-NLOS 样本（有ΔL）: {len(rows)}')

dd_abs = np.array([r['dd_abs'] for r in rows])
print(f'|dd_resid| 分布: 中位={np.median(dd_abs):.1f}m  '
      f'75th={np.percentile(dd_abs,75):.1f}  90th={np.percentile(dd_abs,90):.1f}m')

# ── 二值化标签 ───────────────────────────────────────────────────────────────
pos = dd_abs > args.thr_hi    # 实测真 NLOS
neg = dd_abs < args.thr_lo    # 实测干净
keep = pos | neg
y = pos[keep].astype(int)
print(f'\n标签（|dd|>{args.thr_hi}=真NLOS, |dd|<{args.thr_lo}=干净, 丢弃中间）:')
print(f'  正类(真NLOS): {y.sum()}   负类(干净): {(y==0).sum()}   '
      f'丢弃(中间): {(~keep).sum()}')
if y.sum() < 20 or (y==0).sum() < 20:
    print('警告：某类样本过少，AUC 不可靠')

# ── 特征矩阵 ─────────────────────────────────────────────────────────────────
def feat_col(name):
    return np.array([r[name] for r in rows])[keep]

elev = feat_col('elev')
# 用中位数填补缺失仰角
elev = np.where(np.isnan(elev), np.nanmedian(elev), elev)
features = {
    'delta_L':   feat_col('delta_L'),
    'incidence': feat_col('incidence'),
    'rho_norm':  np.log(np.clip(feat_col('rho_norm'), 1e-3, None)),  # log(ρ)
    'elevation': elev,
}

# ── 单特征 AUC ───────────────────────────────────────────────────────────────
print(f'\n── 单特征 AUC（判别实测真NLOS）──')
for name, x in features.items():
    # AUC 对单调变换不变；方向自动取 max(auc, 1-auc)
    auc = roc_auc_score(y, x)
    auc = max(auc, 1 - auc)
    print(f'  {name:11s}: AUC = {auc:.3f}')

# ── 多特征交叉验证 AUC ───────────────────────────────────────────────────────
X = np.column_stack(list(features.values()))
Xs = StandardScaler().fit_transform(X)
cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=args.seed)

lr = LogisticRegression(max_iter=1000)
rf = RandomForestClassifier(n_estimators=200, max_depth=4,
                             min_samples_leaf=15, random_state=args.seed)
auc_lr = cross_val_score(lr, Xs, y, cv=cv, scoring='roc_auc')
auc_rf = cross_val_score(rf, X,  y, cv=cv, scoring='roc_auc')

print(f'\n── 多特征 5折交叉验证 AUC ──')
print(f'  逻辑回归:   {auc_lr.mean():.3f} ± {auc_lr.std():.3f}')
print(f'  随机森林:   {auc_rf.mean():.3f} ± {auc_rf.std():.3f}')

# RF 特征重要性（全量拟合）
rf.fit(X, y)
print(f'\n  随机森林特征重要性:')
for name, imp in sorted(zip(features.keys(), rf.feature_importances_),
                        key=lambda t: -t[1]):
    print(f'    {name:11s}: {imp:.3f}')

# ── 裁决 ─────────────────────────────────────────────────────────────────────
best_auc = max(auc_lr.mean(), auc_rf.mean())
print(f'\n{"="*60}')
print('裁决')
print(f'{"="*60}')
print(f'  最佳多特征 AUC = {best_auc:.3f}')
if best_auc > 0.65:
    print('  ✅ 方向 B：LiDAR 特征确实带 NLOS 判别信号')
    print('     → 硬阈值光投不是最优；可训练分类器细化 → 有救')
    print('     → 下一步：用 DD 真值训练 LiDAR-NLOS 分类器（诚实标签）')
elif best_auc > 0.58:
    print('  🔶 边界：LiDAR 特征有微弱判别力')
    print('     → 信号存在但弱；可作为辅助特征，难独立成模型')
else:
    print('  ⚠️ 方向 A：LiDAR 特征几乎无判别力 (AUC≈0.5)')
    print('     → LiDAR 几何遮挡与实测 NLOS 偏差无对应，是固有局限')
    print('     → 收口：把严谨阴性结果整理成论文（方法+边界+机理）')
