#!/usr/bin/env python3
"""
lidar_step8g_residualize_elevation.py
最后一击：LiDAR 特征到底是"独立信号"还是"仰角的代理"？

step8f 发现：
  - 加 LiDAR 对全模型零增量（M3−M1≈0）
  - 但低仰角层内 LiDAR-only AUC=0.75（看似有信号）
矛盾点：层内 0.75 可能只是残余仰角变化，而非真 LiDAR 贡献。

本脚本三个决定性测试：
  (1) LiDAR 特征 vs 仰角的相关性（证明是否为代理）
  (2) 残差化：每个 LiDAR 特征对仰角做回归取残差，
      纯残差能否判别 NLOS？残差 AUC≈0.5 → 纯代理
  (3) 低仰角带内：M1(仅仰角) vs M3(仰角+LiDAR) 增量
      （在 NLOS 最常发生的低仰角域，LiDAR 是否真有条件贡献）

输入:  --data dd_resid_material.csv
用法:
  python3 lidar_step8g_residualize_elevation.py \
    --data /root/dd_resid_material.csv --thr_hi 20 --thr_lo 5
"""

import argparse, csv
import numpy as np
from scipy import stats
from sklearn.linear_model import LogisticRegression, LinearRegression
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import StratifiedKFold, cross_val_score
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
            dd = float(r['dd_resid'])
            dL = float(r['delta_L']) if r.get('delta_L') not in (None,'','nan') else np.nan
            ev = float(r['elev'])    if r.get('elev')    not in (None,'','nan') else np.nan
            if np.isnan(dL) or np.isnan(ev): continue
            rows.append({
                'dd_abs':   abs(dd),
                'delta_L':  dL,
                'incidence':float(r['incidence']) if r.get('incidence') else 45.0,
                'rho_norm': float(r['rho_norm']),
                'elev':     ev,
            })
        except (ValueError, KeyError):
            continue

dd_abs = np.array([r['dd_abs'] for r in rows])
pos  = dd_abs > args.thr_hi
neg  = dd_abs < args.thr_lo
keep = pos | neg
y    = pos[keep].astype(int)
print(f'样本（有仰角+ΔL）: {keep.sum()}  正类: {y.sum()}  负类: {(y==0).sum()}')

def col(name):
    return np.array([r[name] for r in rows])[keep]

elev = col('elev')
lidar_raw = {
    'delta_L':   col('delta_L'),
    'incidence': col('incidence'),
    'rho_norm':  np.log(np.clip(col('rho_norm'), 1e-3, None)),
}

# ── 测试1：LiDAR 特征 vs 仰角相关性 ─────────────────────────────────────────
print(f'\n{"="*62}')
print('测试1：LiDAR 特征与仰角的相关性（高相关=代理嫌疑）')
print(f'{"="*62}')
for name, x in lidar_raw.items():
    rsp = stats.spearmanr(x, elev)[0]
    rpe = stats.pearsonr(x, elev)[0]
    flag = '← 强相关(代理嫌疑)' if abs(rsp) > 0.5 else ''
    print(f'  {name:11s}: Spearman={rsp:+.3f}  Pearson={rpe:+.3f}  {flag}')

# ── 测试2：残差化后的 LiDAR 判别力 ───────────────────────────────────────────
print(f'\n{"="*62}')
print('测试2：剥掉仰角后的纯 LiDAR 残差能否判别 NLOS')
print('（对每个 LiDAR 特征用 elevation+elevation^2 回归，取残差）')
print(f'{"="*62}')
E = np.column_stack([elev, elev**2])
lidar_resid = {}
for name, x in lidar_raw.items():
    lr = LinearRegression().fit(E, x)
    resid = x - lr.predict(E)
    lidar_resid[name] = resid
    r2 = lr.score(E, x)
    # 单特征残差 AUC
    from sklearn.metrics import roc_auc_score
    auc = roc_auc_score(y, resid); auc = max(auc, 1-auc)
    print(f'  {name:11s}: 被仰角解释 R²={r2:.3f}  残差单特征AUC={auc:.3f}')

# 残差多特征 AUC
Xr = np.column_stack(list(lidar_resid.values()))
Xrs = StandardScaler().fit_transform(Xr)
cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=args.seed)
rf = RandomForestClassifier(n_estimators=200, max_depth=4,
                            min_samples_leaf=15, random_state=args.seed)
lr_clf = LogisticRegression(max_iter=1000)
auc_r_rf = cross_val_score(rf, Xr,  y, cv=cv, scoring='roc_auc')
auc_r_lr = cross_val_score(lr_clf, Xrs, y, cv=cv, scoring='roc_auc')
print(f'\n  纯LiDAR残差 多特征AUC:  LogReg={auc_r_lr.mean():.3f}±{auc_r_lr.std():.3f}  '
      f'RF={auc_r_rf.mean():.3f}±{auc_r_rf.std():.3f}')

# ── 测试3：低仰角带内的真实增量 ──────────────────────────────────────────────
print(f'\n{"="*62}')
print('测试3：低仰角带（<33th 分位）内 M1(仰角) vs M3(仰角+LiDAR) 增量')
print('（NLOS 最常发生处，控制残余仰角后 LiDAR 是否仍有条件贡献）')
print(f'{"="*62}')
lo_thr = np.percentile(elev, 33)
mask = elev < lo_thr
ym = y[mask]
print(f'  低仰角带: n={mask.sum()}  正类={ym.sum()}  负类={(ym==0).sum()}')
if ym.sum() >= 30 and (ym==0).sum() >= 30:
    elev_m = elev[mask]
    X1 = np.column_stack([elev_m])
    X3 = np.column_stack([elev_m] + [v[mask] for v in lidar_raw.values()])
    cv_m = StratifiedKFold(n_splits=5, shuffle=True, random_state=args.seed)
    for clf_name, clf in [('RF', RandomForestClassifier(
                                n_estimators=200, max_depth=4,
                                min_samples_leaf=15, random_state=args.seed)),
                          ('LogReg', LogisticRegression(max_iter=1000))]:
        if clf_name == 'LogReg':
            a1 = cross_val_score(clf, StandardScaler().fit_transform(X1), ym,
                                 cv=cv_m, scoring='roc_auc')
            a3 = cross_val_score(clf, StandardScaler().fit_transform(X3), ym,
                                 cv=cv_m, scoring='roc_auc')
        else:
            a1 = cross_val_score(clf, X1, ym, cv=cv_m, scoring='roc_auc')
            a3 = cross_val_score(clf, X3, ym, cv=cv_m, scoring='roc_auc')
        print(f'  [{clf_name}] 仅仰角={a1.mean():.3f}  仰角+LiDAR={a3.mean():.3f}  '
              f'增量={a3.mean()-a1.mean():+.3f}')
else:
    print('  样本不足，跳过')

# ── 裁决 ─────────────────────────────────────────────────────────────────────
best_resid = max(auc_r_rf.mean(), auc_r_lr.mean())
print(f'\n{"="*62}')
print('裁决')
print(f'{"="*62}')
print(f'  纯 LiDAR 残差（剥离仰角后）多特征 AUC = {best_resid:.3f}')
if best_resid > 0.62:
    print('  ✅ 残差仍有判别力 → LiDAR 携带独立于仰角的真信号')
    print('     → 研究有救：LiDAR 在仰角之外贡献 NLOS 信息')
    print('     → 即使全模型增量被仰角吸收，机理上 LiDAR 有效')
elif best_resid > 0.55:
    print('  🔶 残差有微弱判别力 → LiDAR 部分独立、部分代理')
    print('     → 条件性结论：特定子域（低仰角深峡谷）LiDAR 有限增益')
else:
    print('  ⚠️ 残差 AUC≈0.5 → LiDAR 特征几乎纯是仰角的代理')
    print('     → 决定性结论：剥离仰角后 LiDAR 无独立 NLOS 判别力')
    print('     → step8e/8f 的表面"成功"全部来自卫星几何（仰角）')
    print('     → 诚实收口：这是严谨的边界/阴性结果，本身有发表价值')
    print('       命题：单帧 LiDAR 几何遮挡 ≠ 实测伪距 NLOS，')
    print('             仰角已是更强且免费的预测因子')
