#!/usr/bin/env python3
"""
lidar_step6h_beidou_orbit_check.py
诊断北斗 severe 零相关是否由 GEO/IGSO 轨道几何混淆造成。

假设：北斗在香港有大量 GEO/IGSO 卫星（高仰角、近静止），其 NLOS 几何
与 MEO 的"建筑物侧面反射"不同。若剔除 GEO/IGSO 后，北斗 MEO 的 severe
相关性变负，则证实北斗零相关是轨道几何混淆，而非材质效应不存在。

北斗 PRN 轨道分类（BDS-2/BDS-3, 2021）：
  GEO : C01-C05
  IGSO: C06-C10, C13, C16
  MEO : 其余（C11,C12,C14,C19+ ...）

输入:
  --data  rho_cn0_analysis_mg.csv  (step6f 输出，含 sys/sat_id/elevation)
输出:
  控制台诊断

用法:
  python3 lidar_step6h_beidou_orbit_check.py --data /root/rho_cn0_analysis_mg.csv
"""

import argparse, csv
import numpy as np
from scipy import stats

ap = argparse.ArgumentParser()
ap.add_argument('--data',   default='/root/rho_cn0_analysis_mg.csv')
ap.add_argument('--n_boot', type=int, default=5000)
ap.add_argument('--seed',   type=int, default=42)
args = ap.parse_args()
rng = np.random.default_rng(args.seed)

GEO  = {f'C{n:02d}' for n in range(1, 6)}          # C01-C05
IGSO = {f'C{n:02d}' for n in [6,7,8,9,10,13,16]}   # C06-C10,C13,C16

def orbit_type(sat_id):
    if sat_id in GEO:  return 'GEO'
    if sat_id in IGSO: return 'IGSO'
    return 'MEO'

# ── 读数据（只取北斗）────────────────────────────────────────────────────────
rows = []
with open(args.data) as f:
    for r in csv.DictReader(f):
        if r.get('sys') != 'C':
            continue
        try:
            rho = float(r['rho_norm'])
            if rho <= 0: continue
            rows.append({
                'sat_id':   r['sat_id'],
                'orbit':    orbit_type(r['sat_id']),
                'severity': r.get('severity',''),
                'rho':      rho,
                'drop':     float(r['cn0_drop']),
                'elev':     float(r['elevation']) if r.get('elevation') else np.nan,
            })
        except (ValueError, KeyError):
            continue

print(f'北斗样本: {len(rows)}')

# ── 轨道类型构成 ─────────────────────────────────────────────────────────────
from collections import Counter
print(f'\n北斗 PRN 构成:')
prn_cnt = Counter(r['sat_id'] for r in rows)
for prn in sorted(prn_cnt):
    print(f'  {prn} ({orbit_type(prn)}): {prn_cnt[prn]}')

print(f'\n按轨道类型:')
print(f'  {"orbit":<6} {"n":>5} {"mean_elev":>10} {"severe_n":>9} {"drop均值":>9}')
for ot in ['GEO','IGSO','MEO']:
    sub = [r for r in rows if r['orbit']==ot]
    if not sub: continue
    elevs = np.array([r['elev'] for r in sub])
    sev_n = sum(1 for r in sub if r['severity']=='severe')
    drops = np.array([r['drop'] for r in sub])
    print(f'  {ot:<6} {len(sub):>5} {np.nanmean(elevs):>10.1f} {sev_n:>9} {drops.mean():>+9.2f}')

# ── severe 相关性：全北斗 vs 仅MEO ───────────────────────────────────────────
def boot_sp(rho, drop):
    if len(rho) < 15: return None
    point = stats.spearmanr(rho, drop)[0]
    boots = np.empty(args.n_boot); idx = np.arange(len(rho))
    for i in range(args.n_boot):
        s = rng.choice(idx, size=len(rho), replace=True)
        boots[i] = stats.spearmanr(rho[s], drop[s])[0]
    lo, hi = np.percentile(boots, [2.5, 97.5])
    p = stats.spearmanr(rho, drop)[1]
    return point, lo, hi, p, float(np.mean(boots<0))

print(f'\n{"="*60}')
print('severe 子集：全北斗 vs 剔除 GEO/IGSO（仅 MEO）')
print(f'{"="*60}')
for label, filt in [('全北斗 severe', lambda r: True),
                    ('北斗MEO severe', lambda r: r['orbit']=='MEO')]:
    sub = [r for r in rows if r['severity']=='severe' and filt(r)]
    if len(sub) < 15:
        print(f'  {label}: n={len(sub)} 样本不足')
        continue
    rho  = np.array([r['rho'] for r in sub])
    drop = np.array([r['drop'] for r in sub])
    res = boot_sp(rho, drop)
    point, lo, hi, p, fn = res
    ci = '✓负' if (lo<0 and hi<0) else ('~偏负' if fn>0.9 else '✗')
    print(f'  {label:<16} n={len(sub):>4}  r={point:+.3f}  '
          f'CI=[{lo:+.3f},{hi:+.3f}]  p={p:.3f}  bootstrap负={100*fn:.0f}%  {ci}')

print(f'\n裁决:')
print('  若"北斗MEO severe"显著变负 → 证实零相关是 GEO/IGSO 几何混淆')
print('  若仍为零 → 北斗材质效应确实较弱（可能 B1I 信号特性）')
