#!/usr/bin/env python3
"""
lidar_step6g_severe_per_constellation.py
决定性诊断：severe NLOS 的"材质→CN0衰减"效应是否在 G/C/E 三个星座中
各自独立成立（跨星座交叉验证）。

逻辑：
  对每个 (severity × constellation) 组合，计算 ρ_norm vs CN0_drop 的
  Spearman r + bootstrap 95% CI。重点看 severe 行：
    - 若 G/C/E 三星座 severe 都为负且 CI 偏向负 → 效应三重交叉验证（硬结论）
    - 若仅 GPS 为负 → 效应弱，需换目标变量或收窄命题

输入:
  --data  rho_cn0_analysis_mg.csv  (step6f 输出，含 sys 列)
输出:
  控制台表格 + --out_csv 汇总

用法:
  python3 lidar_step6g_severe_per_constellation.py \\
    --data /root/rho_cn0_analysis_mg.csv \\
    --out_csv /root/severe_per_constellation.csv
"""

import argparse, csv
import numpy as np
from scipy import stats

ap = argparse.ArgumentParser()
ap.add_argument('--data',    default='/root/rho_cn0_analysis_mg.csv')
ap.add_argument('--out_csv', default='/root/severe_per_constellation.csv')
ap.add_argument('--n_boot',  type=int, default=5000)
ap.add_argument('--seed',    type=int, default=42)
args = ap.parse_args()

rng = np.random.default_rng(args.seed)

# ── 读数据 ────────────────────────────────────────────────────────────────────
rows = []
with open(args.data) as f:
    for r in csv.DictReader(f):
        try:
            if r.get('rho_norm', '') == '' or r.get('cn0_drop', '') == '':
                continue
            rho = float(r['rho_norm'])
            if rho <= 0:
                continue
            rows.append({
                'sys':      r.get('sys', r['sat_id'][0]),
                'severity': r.get('severity', ''),
                'rho':      rho,
                'drop':     float(r['cn0_drop']),
            })
        except (ValueError, KeyError):
            continue

print(f'有效样本: {len(rows)}')

def boot_spearman(rho, drop):
    if len(rho) < 15:
        return None
    point = stats.spearmanr(rho, drop)[0]
    boots = np.empty(args.n_boot)
    idx = np.arange(len(rho))
    for i in range(args.n_boot):
        s = rng.choice(idx, size=len(rho), replace=True)
        boots[i] = stats.spearmanr(rho[s], drop[s])[0]
    lo, hi = np.percentile(boots, [2.5, 97.5])
    frac_neg = float(np.mean(boots < 0))
    _, p = stats.spearmanr(rho, drop)
    return point, lo, hi, p, frac_neg

# ── 逐 (severity × sys) 计算 ─────────────────────────────────────────────────
results = []
print(f'\n{"severity":<8} {"sys":<4} {"n":>5}  {"Spearman r":>11}  '
      f'{"95% CI":>20}  {"p":>8}  {"r<0占比":>8}')
print('-' * 78)

for sev in ['severe', 'strong', 'mild']:
    for sysc in ['G', 'C', 'E']:
        sub = [r for r in rows if r['severity'] == sev and r['sys'] == sysc]
        if len(sub) < 15:
            print(f'{sev:<8} {sysc:<4} {len(sub):>5}  (样本不足，跳过)')
            continue
        rho  = np.array([r['rho'] for r in sub])
        drop = np.array([r['drop'] for r in sub])
        res = boot_spearman(rho, drop)
        point, lo, hi, p, frac_neg = res
        sig = '*' if p < 0.05 else ' '
        ci_neg = '✓负' if (lo < 0 and hi < 0) else ('~' if lo < 0 else '✗正')
        print(f'{sev:<8} {sysc:<4} {len(sub):>5}  {point:>+11.3f}{sig}  '
              f'[{lo:+.3f}, {hi:+.3f}]  {p:>8.4f}  {100*frac_neg:>6.1f}%  {ci_neg}')
        results.append({
            'severity': sev, 'sys': sysc, 'n': len(sub),
            'r': round(point, 4), 'ci_lo': round(lo, 4), 'ci_hi': round(hi, 4),
            'p': round(p, 4), 'frac_neg': round(frac_neg, 3),
        })
    print()

# ── severe 行的裁决 ──────────────────────────────────────────────────────────
print('=' * 60)
print('severe 子集跨星座裁决')
print('=' * 60)
sev_res = [r for r in results if r['severity'] == 'severe']
n_neg      = sum(1 for r in sev_res if r['r'] < 0)
n_neg_strong = sum(1 for r in sev_res if r['frac_neg'] > 0.9)  # bootstrap 90%+ 为负
print(f'  severe 有 {len(sev_res)} 个星座有足够样本')
print(f'  其中 r<0 的: {n_neg}/{len(sev_res)}')
print(f'  其中 bootstrap 90%+ 偏负的: {n_neg_strong}/{len(sev_res)}')

if len(sev_res) >= 2 and n_neg == len(sev_res) and n_neg_strong >= 2:
    print('\n  ✅ 裁决：severe 效应跨星座交叉验证成立')
    print('     → 收窄命题成立：「长路径 NLOS 中材质主导信号衰减」是硬结论')
    print('     → 建议方向 A：写条件性材质效应论文')
elif n_neg >= 1:
    print('\n  🔶 裁决：severe 效应主要由部分星座驱动（可能 GPS 主导）')
    print('     → 效应真实但偏弱，建议考虑换目标变量（伪距误差）')
    print('     → 或方向 C：把"星座调制依赖 NLOS 衰减"做成新角度')
else:
    print('\n  ⚠️ 裁决：severe 效应未跨星座复现')
    print('     → 原 CN0 目标可能不适合，建议方向 B：材质降为辅助特征')

# ── 写 CSV ────────────────────────────────────────────────────────────────────
with open(args.out_csv, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['severity','sys','n','r','ci_lo','ci_hi','p','frac_neg'])
    w.writeheader()
    w.writerows(results)
print(f'\n写出: {args.out_csv}')
