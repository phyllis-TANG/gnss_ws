#!/usr/bin/env python3
"""
lidar_step9a_within_sat_temporal_premise.py
多历元前提检验：把"跨卫星"换成"同一颗卫星沿自己轨迹"，看几何/材质是否还有命。

单帧诊断（step8）把所有卫星混在一起，ΔL vs dd_resid 只有 −0.062。但那混入了
"卫星身份"这个混淆（CV 泄漏的根源）。真正该问的是——
  同一颗星在移动、它的 ΔL/ρ 在变，实测 dd_resid 跟不跟着变？
这是任何几何改正 / 材质衰减模型的物理前提。

三个测试：
  (1) 星内去均值相关：每颗卫星减去自身均值后，dd_resid vs ΔL / vs log(ρ_norm)
      —— 控制了卫星身份，是诚实的"星内物理"信号
  (2) 时序平滑：每颗星沿时间滑动平均后再相关 —— 若单帧噪声掩盖了信号，平滑会显现
  (3) NLOS 持续性 run-length：|dd_resid|>阈值 连续持续多少历元
      —— 持续越长，时序模型（平滑/HMM/降权）越能利用

裁决：
  星内 ΔL↔dd > 0.2  → 几何路径星内有效 → 多历元改正有戏（押③伪距改正）
  星内 ρ↔dd  显著   → 材质在星内有效 → 材质衰减模型有戏（课题核心方向）
  两者都≈0          → 几何/材质星内都无效 → 只能退回①检测/加权或收口

输入:  --data dd_resid_material.csv （含 utc_t, sat_id, dd_resid, delta_L, rho_norm）
用法:
  python3 lidar_step9a_within_sat_temporal_premise.py \
    --data /root/dd_resid_material.csv --thr 20 --win 5
"""

import argparse, csv
import numpy as np
from collections import defaultdict
from scipy import stats

ap = argparse.ArgumentParser()
ap.add_argument('--data', default='/root/dd_resid_material.csv')
ap.add_argument('--thr',  type=float, default=20.0, help='|dd_resid|>此值算NLOS（持续性用）')
ap.add_argument('--win',  type=int,   default=5,    help='时序平滑窗口（历元）')
ap.add_argument('--gap',  type=float, default=2.0,  help='相邻历元时间差<此值才算连续(秒)')
ap.add_argument('--min_n',type=int,   default=8,    help='每颗卫星最少历元数才纳入星内分析')
args = ap.parse_args()

# ── 读数据，按卫星分组 ────────────────────────────────────────────────────────
by_sat = defaultdict(list)
n_raw = 0
with open(args.data) as f:
    for r in csv.DictReader(f):
        try:
            dL = r.get('delta_L')
            if dL in (None, '', 'nan'):  # 只看 LiDAR-NLOS（有几何路径）的卫星
                continue
            rec = {
                'utc':  float(r['utc_t']),
                'dd':   float(r['dd_resid']),
                'dL':   float(dL),
                'rho':  float(r['rho_norm']) if r.get('rho_norm') not in (None,'','nan') else np.nan,
            }
            by_sat[r['sat_id']].append(rec)
            n_raw += 1
        except (ValueError, KeyError):
            continue

print(f'LiDAR-NLOS 样本: {n_raw}   卫星数: {len(by_sat)}')

# 每颗星按时间排序
for s in by_sat:
    by_sat[s].sort(key=lambda x: x['utc'])

# ── 基线：跨卫星混合（复现单帧 −0.06）─────────────────────────────────────────
all_dd  = np.array([rec['dd']  for s in by_sat for rec in by_sat[s]])
all_dL  = np.array([rec['dL']  for s in by_sat for rec in by_sat[s]])
all_rho = np.array([rec['rho'] for s in by_sat for rec in by_sat[s]])
m = ~np.isnan(all_rho)
print(f'\n{"="*64}')
print('基线：跨卫星混合相关（含卫星身份混淆，应≈单帧结果）')
print(f'{"="*64}')
print(f'  dd vs ΔL      : Spearman={stats.spearmanr(all_dd, all_dL)[0]:+.3f}')
print(f'  dd vs log(ρ)  : Spearman={stats.spearmanr(all_dd[m], np.log(np.clip(all_rho[m],1e-3,None)))[0]:+.3f}')

# ── 测试1：星内去均值相关 ─────────────────────────────────────────────────────
def within_demean(get_x, need_rho=False):
    """每颗卫星减自身均值后池化，返回 (x_resid, dd_resid_pooled)"""
    xs, ds = [], []
    for s, recs in by_sat.items():
        if len(recs) < args.min_n:
            continue
        x  = np.array([get_x(rec) for rec in recs], float)
        dd = np.array([rec['dd'] for rec in recs], float)
        ok = ~np.isnan(x) & ~np.isnan(dd)
        if ok.sum() < args.min_n:
            continue
        x, dd = x[ok], dd[ok]
        xs.append(x - x.mean())
        ds.append(dd - dd.mean())
    if not xs:
        return None
    return np.concatenate(xs), np.concatenate(ds)

print(f'\n{"="*64}')
print('测试1：星内去均值相关（控制卫星身份 → 诚实的星内物理信号）')
print(f'{"="*64}')
for label, getx, need_rho in [
        ('dd vs ΔL',     lambda r: r['dL'], False),
        ('dd vs log(ρ)', lambda r: np.log(np.clip(r['rho'],1e-3,None)) if not np.isnan(r['rho']) else np.nan, True)]:
    res = within_demean(getx, need_rho)
    if res is None:
        print(f'  {label:<14}: 样本不足'); continue
    xr, dr = res
    sp = stats.spearmanr(xr, dr)
    pe = stats.pearsonr(xr, dr)
    print(f'  {label:<14}: n={len(xr):>5}  Spearman={sp[0]:+.3f} (p={sp[1]:.3f})  '
          f'Pearson={pe[0]:+.3f}')

# ── 测试2：时序平滑后星内相关 ────────────────────────────────────────────────
def smooth(arr, w):
    if len(arr) < w: return arr.copy()
    k = np.ones(w)/w
    return np.convolve(arr, k, mode='same')

print(f'\n{"="*64}')
print(f'测试2：时序平滑（窗口={args.win}历元）后星内去均值相关')
print(f'（若单帧噪声掩盖了信号，平滑后相关应增强）')
print(f'{"="*64}')
for label, key, islog in [('dd vs ΔL','dL',False), ('dd vs log(ρ)','rho',True)]:
    xs, ds = [], []
    for s, recs in by_sat.items():
        if len(recs) < max(args.min_n, args.win):
            continue
        x  = np.array([rec[key] for rec in recs], float)
        dd = np.array([rec['dd'] for rec in recs], float)
        if islog:
            x = np.log(np.clip(x, 1e-3, None))
        ok = ~np.isnan(x) & ~np.isnan(dd)
        if ok.sum() < max(args.min_n, args.win):
            continue
        x, dd = x[ok], dd[ok]
        xsm, dsm = smooth(x, args.win), smooth(dd, args.win)
        xs.append(xsm - xsm.mean())
        ds.append(dsm - dsm.mean())
    if not xs:
        print(f'  {label:<14}: 样本不足'); continue
    xr, dr = np.concatenate(xs), np.concatenate(ds)
    sp = stats.spearmanr(xr, dr)
    print(f'  {label:<14}: n={len(xr):>5}  Spearman={sp[0]:+.3f} (p={sp[1]:.3f})')

# ── 测试3：NLOS 持续性 run-length ────────────────────────────────────────────
print(f'\n{"="*64}')
print(f'测试3：NLOS 持续性（|dd_resid|>{args.thr}m 连续 run-length）')
print(f'（持续越长，时序模型/降权越能利用；≈1 则逐历元随机，时序无益）')
print(f'{"="*64}')
runs = []
nlos_frac = []
for s, recs in by_sat.items():
    times = np.array([rec['utc'] for rec in recs])
    flag  = np.array([abs(rec['dd']) > args.thr for rec in recs])
    nlos_frac.append(flag.mean())
    # 按时间连续性切段，统计连续 NLOS run
    cur = 0
    for i in range(len(flag)):
        if i > 0 and (times[i] - times[i-1]) > args.gap:
            if cur > 0: runs.append(cur)
            cur = 0
        if flag[i]:
            cur += 1
        else:
            if cur > 0: runs.append(cur)
            cur = 0
    if cur > 0: runs.append(cur)

if runs:
    runs = np.array(runs)
    print(f'  NLOS run 段数: {len(runs)}')
    print(f'  run 长度: 中位={np.median(runs):.0f}  均值={runs.mean():.1f}  '
          f'最长={runs.max()}  ≥5历元的占比={np.mean(runs>=5)*100:.0f}%')
    print(f'  各卫星 NLOS 时间占比: 中位={np.median(nlos_frac)*100:.0f}%')
else:
    print('  无 NLOS run')

# ── 裁决 ─────────────────────────────────────────────────────────────────────
res_dL  = within_demean(lambda r: r['dL'])
res_rho = within_demean(lambda r: np.log(np.clip(r['rho'],1e-3,None)) if not np.isnan(r['rho']) else np.nan)
sp_dL  = abs(stats.spearmanr(*res_dL)[0])  if res_dL  else 0.0
sp_rho = abs(stats.spearmanr(*res_rho)[0]) if res_rho else 0.0

print(f'\n{"="*64}')
print('裁决')
print(f'{"="*64}')
print(f'  星内 |dd↔ΔL|   = {sp_dL:.3f}')
print(f'  星内 |dd↔log ρ| = {sp_rho:.3f}')
best = max(sp_dL, sp_rho)
persist_ok = bool(len(runs)) and np.median(runs) >= 3

if best > 0.2:
    which = 'ΔL（几何路径）' if sp_dL >= sp_rho else 'ρ_norm（材质）'
    print(f'\n  ✅ 星内相关显著（{which} 主导）→ 多历元/物理建模有真前提')
    print('     → 值得下注：建带时序的改正/加权模型')
    if sp_rho > 0.2:
        print('     → 且材质 ρ 在星内有效 → 课题核心(材质衰减)方向活着')
elif best > 0.1:
    print('\n  🔶 星内有弱相关 → 几何/材质在星内携带有限信号')
    print('     → 押①：做时序 NLOS 检测/降权（不押伪距改正③）')
    if persist_ok:
        print(f'     → NLOS 持续性足够(中位 run≥3)，时序降权可利用')
else:
    print('\n  ⚠️ 星内相关也≈0 → 几何/材质连在单星轨迹内都追不上实测偏差')
    print('     → 多历元救不了改正；时序顶多帮"检测"靠持续性，非靠物理')
    if persist_ok:
        print(f'     → 但 NLOS 持续性存在 → 纯时序降权(不依赖LiDAR物理)仍可能小幅改善定位')
    print('     → 建议：收口写阴性+方法论，LiDAR物理这条按当前数据到此')
