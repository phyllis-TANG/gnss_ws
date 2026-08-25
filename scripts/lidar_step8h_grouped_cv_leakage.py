#!/usr/bin/env python3
"""
lidar_step8h_grouped_cv_leakage.py
决定性泄漏检验：step8g 的 RF 残差 AUC=0.748 是真信号还是自相关泄漏？

step8g 矛盾点：
  - 单特征残差 AUC 全 ~0.55（弱）
  - 但 RF 多特征残差 AUC=0.748，LogReg 只 0.597
  - 怀疑：同卫星连续历元高度相关，随机 K-fold 把相关簇切到训练+测试两边，
          RF 靠记忆作弊 → AUC 虚高

本脚本对比三种交叉验证切分：
  (A) 随机 KFold        —— 会泄漏（复现 step8g 的虚高）
  (B) GroupKFold by 卫星 —— 测"能否泛化到没见过的卫星"
  (C) GroupKFold by 时间块(60s) —— 相关历元保持同侧，测"泛化到没见过的时段"

若 RF 在 (B)(C) 下塌回 ~0.55-0.60（接近 LogReg/单特征）→ 0.748 是泄漏。
若 RF 在分组下仍 >0.65 → LiDAR 真有可泛化的独立信号。

输入:  --data dd_resid_material.csv （含 utc_t, sat_id）
用法:
  python3 lidar_step8h_grouped_cv_leakage.py \
    --data /root/dd_resid_material.csv --thr_hi 20 --thr_lo 5
"""

import argparse, csv
import numpy as np
from sklearn.linear_model import LogisticRegression, LinearRegression
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import (StratifiedKFold, GroupKFold, cross_val_score)
from sklearn.preprocessing import StandardScaler

ap = argparse.ArgumentParser()
ap.add_argument('--data',   default='/root/dd_resid_material.csv')
ap.add_argument('--thr_hi', type=float, default=20.0)
ap.add_argument('--thr_lo', type=float, default=5.0)
ap.add_argument('--block',  type=float, default=60.0, help='时间块长度(秒)')
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
                'sat_id':   r['sat_id'],
                'utc_t':    float(r['utc_t']) if r.get('utc_t') not in (None,'','nan') else np.nan,
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
idx  = np.where(keep)[0]
print(f'样本: {keep.sum()}  正类: {y.sum()}  负类: {(y==0).sum()}')

def col(name):
    return np.array([rows[i][name] for i in idx])

elev = col('elev')
sat  = col('sat_id')
utc  = col('utc_t')
lidar_raw = {
    'delta_L':   col('delta_L'),
    'incidence': col('incidence'),
    'rho_norm':  np.log(np.clip(col('rho_norm'), 1e-3, None)),
}

# ── 残差化（剥离仰角，与 step8g 一致）────────────────────────────────────────
E = np.column_stack([elev, elev**2])
lidar_resid = {}
for name, x in lidar_raw.items():
    lr = LinearRegression().fit(E, x)
    lidar_resid[name] = x - lr.predict(E)
Xr  = np.column_stack(list(lidar_resid.values()))
Xrs = StandardScaler().fit_transform(Xr)

# ── 分组定义 ─────────────────────────────────────────────────────────────────
# 卫星分组
groups_sat = sat
# 时间块分组：sat_id × floor(utc/block)，相关历元同块
t0 = np.nanmin(utc)
blk = np.where(np.isnan(utc), -1, np.floor((utc - t0) / args.block)).astype(int)
groups_time = np.array([f'{s}_{b}' for s, b in zip(sat, blk)])

print(f'\n唯一卫星数: {len(set(sat))}   唯一时间块(sat×{int(args.block)}s): {len(set(groups_time))}')

# ── 三种 CV 对比 ─────────────────────────────────────────────────────────────
rf = RandomForestClassifier(n_estimators=200, max_depth=4,
                            min_samples_leaf=15, random_state=args.seed)
lr_clf = LogisticRegression(max_iter=1000)

def run_cv(splitter, groups, label):
    try:
        if groups is None:
            a_rf = cross_val_score(rf, Xr,  y, cv=splitter, scoring='roc_auc')
            a_lr = cross_val_score(lr_clf, Xrs, y, cv=splitter, scoring='roc_auc')
        else:
            a_rf = cross_val_score(rf, Xr,  y, groups=groups, cv=splitter, scoring='roc_auc')
            a_lr = cross_val_score(lr_clf, Xrs, y, groups=groups, cv=splitter, scoring='roc_auc')
        print(f'  {label:<28}  RF={a_rf.mean():.3f}±{a_rf.std():.3f}   '
              f'LogReg={a_lr.mean():.3f}±{a_lr.std():.3f}')
        return a_rf.mean(), a_lr.mean()
    except Exception as e:
        print(f'  {label:<28}  失败: {e}')
        return None, None

print(f'\n{"="*68}')
print('纯 LiDAR 残差（剥离仰角）在三种 CV 切分下的 AUC')
print(f'{"="*68}')
rf_a, lr_a = run_cv(StratifiedKFold(5, shuffle=True, random_state=args.seed), None,
                    '(A) 随机KFold[会泄漏]')
rf_b, lr_b = run_cv(GroupKFold(n_splits=5), groups_sat,
                    '(B) GroupKFold/卫星')
rf_c, lr_c = run_cv(GroupKFold(n_splits=5), groups_time,
                    f'(C) GroupKFold/时间块{int(args.block)}s')

# ── 裁决 ─────────────────────────────────────────────────────────────────────
print(f'\n{"="*68}')
print('裁决')
print(f'{"="*68}')
print(f'  随机KFold  RF AUC = {rf_a:.3f}  (step8g 复现，含泄漏)')
if rf_b is not None:
    print(f'  分组/卫星  RF AUC = {rf_b:.3f}')
if rf_c is not None:
    print(f'  分组/时间  RF AUC = {rf_c:.3f}')

grouped_best = max(v for v in [rf_b, rf_c, lr_b, lr_c] if v is not None)
drop = rf_a - (max(rf_b or 0, rf_c or 0))
print(f'\n  分组后最佳 AUC = {grouped_best:.3f}   RF 跌幅(随机−分组) = {drop:+.3f}')

if grouped_best > 0.65:
    print('\n  ✅ 分组 CV 下仍 >0.65 → LiDAR 携带可泛化的独立 NLOS 信号')
    print('     → 真有救：信号能泛化到没见过的卫星/时段')
    print('     → 下一步：正式训练 LiDAR-aided NLOS 检测器，报告分组 AUC')
elif grouped_best > 0.58:
    print('\n  🔶 分组 CV 下 0.58-0.65 → 有弱但真实的可泛化信号')
    print('     → 条件性结论：LiDAR 作辅助特征，增量有限')
    print(f'     → 注意：随机KFold的0.748有 {drop:+.3f} 是泄漏虚高')
else:
    print('\n  ⚠️ 分组 CV 下塌回 ≈0.5-0.58 → step8g 的 0.748 主要是自相关泄漏')
    print(f'     → RF 跌幅 {drop:+.3f}：随机KFold 把同卫星相关历元切两边，RF 记忆作弊')
    print('     → 决定性诚实结论：LiDAR 几何/材质特征无可泛化的独立 NLOS 判别力')
    print('     → 收口：严谨阴性/边界结果（方法论上尤其有价值——揭示该领域')
    print('       常见的随机CV泄漏陷阱），整理成论文')
