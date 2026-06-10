#!/usr/bin/env python3
"""
lidar_step6b_extract_intensity.py
从带 intensity 的 ENU 点云地图中，为每个 LiDAR NLOS 反射命中点提取
表面 LiDAR 强度，并归一化为材质反射率 ρ_norm。

物理模型（激光雷达方程简化，参考 Jutzi & Stilla 2006）：
    I_measured = η × ρ × cos(α) / R²
    → ρ_norm = I_measured × R² / (η_ref × cos(α))
       其中 η_ref 归一化为全数据集 η_ref = median(I × R² / cos(α))，
       使 ρ_norm 的中位数 = 1.0（相对反射率）

输入:
  --pcd   urbannav_map_intensity.pcd   (step2b 输出，4列: x y z intensity)
  --refl  lidar_reflection_model.csv   (step6 输出，含 hit_enu_e/n/u, hit_dist_m, incidence_deg)

输出:
  --out   lidar_reflection_intensity.csv
          新增列: intensity, rho_norm, nn_dist_m (最近邻距离，质量指标)

用法:
  python3 lidar_step6b_extract_intensity.py \
    --pcd  /root/urbannav_map_intensity.pcd \
    --refl /root/lidar_reflection_model.csv \
    --out  /root/lidar_reflection_intensity.csv \
    --radius 0.8
"""

import argparse, csv, struct, math, os, time
import numpy as np
from scipy.spatial import cKDTree

ap = argparse.ArgumentParser()
ap.add_argument('--pcd',    default='/root/urbannav_map_intensity.pcd')
ap.add_argument('--refl',   default='/root/lidar_reflection_model.csv')
ap.add_argument('--out',    default='/root/lidar_reflection_intensity.csv')
ap.add_argument('--radius', type=float, default=0.8,
                help='最近邻搜索半径（米）；超出则 intensity=NaN（默认 0.8m）')
args = ap.parse_args()

# ── 读 PCD（x y z intensity，二进制）────────────────────────────────────────
def read_pcd_xyzi(path):
    with open(path, 'rb') as f:
        n_points = 0
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('POINTS'):
                n_points = int(line.split()[1])
            if line.startswith('DATA'):
                break
        raw = f.read(n_points * 4 * 4)   # 4 fields × 4 bytes
    arr = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4)
    return arr   # columns: x, y, z, intensity

print(f'读取点云: {args.pcd}')
t0 = time.time()
cloud = read_pcd_xyzi(args.pcd)
print(f'  {len(cloud):,} 点，intensity {cloud[:,3].min():.1f}~{cloud[:,3].max():.1f}，'
      f'耗时 {time.time()-t0:.1f}s')

print('构建 KD-tree...')
t0 = time.time()
tree = cKDTree(cloud[:, :3])
print(f'  完成，耗时 {time.time()-t0:.1f}s')

# ── 读 reflection model ──────────────────────────────────────────────────────
print(f'读取反射模型: {args.refl}')
rows = []
with open(args.refl) as f:
    rows = list(csv.DictReader(f))
print(f'  {len(rows)} 条记录')

# ── 查询每个 hit 点的 intensity ───────────────────────────────────────────────
print(f'KD-tree 查询（半径={args.radius}m）...')
t0 = time.time()

hit_pts = np.array([
    [float(r['hit_enu_e']), float(r['hit_enu_n']), float(r['hit_enu_u'])]
    for r in rows
], dtype=np.float32)

dists, idxs = tree.query(hit_pts, k=1, workers=-1)

intensities = np.where(dists <= args.radius, cloud[idxs, 3], np.nan)
print(f'  命中（<{args.radius}m）: {np.sum(~np.isnan(intensities)):,} / {len(rows)} '
      f'({100*np.mean(~np.isnan(intensities)):.1f}%)')
print(f'  耗时 {time.time()-t0:.1f}s')

# ── 归一化：ρ_norm = I × R² / (η_ref × cos(α)) ──────────────────────────────
hit_dist  = np.array([float(r['hit_dist_m'])    for r in rows], dtype=np.float64)
incidence = np.array([float(r['incidence_deg']) for r in rows], dtype=np.float64)

cos_alpha = np.cos(np.radians(incidence))
cos_alpha = np.clip(cos_alpha, 0.05, 1.0)   # 避免除零（入射角接近 90°）

# 原始"反射率项"（未归一化）
raw_rho = intensities * (hit_dist ** 2) / cos_alpha   # I × R² / cos(α)

# η_ref = 有效点的中位数（使 ρ_norm 中位数 ≈ 1.0）
valid_mask = ~np.isnan(raw_rho)
if valid_mask.sum() > 0:
    eta_ref = np.nanmedian(raw_rho[valid_mask])
    rho_norm = raw_rho / eta_ref
    print(f'\nη_ref = {eta_ref:.2f}')
    print(f'ρ_norm 统计（有效点）:')
    print(f'  中位数: {np.nanmedian(rho_norm):.3f}')
    print(f'  均值:   {np.nanmean(rho_norm):.3f}')
    print(f'  std:    {np.nanstd(rho_norm):.3f}')
    print(f'  5th:    {np.nanpercentile(rho_norm, 5):.3f}')
    print(f'  95th:   {np.nanpercentile(rho_norm, 95):.3f}')
else:
    eta_ref = 1.0
    rho_norm = raw_rho
    print('警告：无有效命中点')

# ── 按 severity 分组统计 ────────────────────────────────────────────────────
print('\nρ_norm 按反射严重程度分组:')
for sev in ['mild', 'strong', 'severe']:
    mask = np.array([r['severity'] == sev for r in rows]) & valid_mask
    if mask.sum() > 0:
        vals = rho_norm[mask]
        print(f'  {sev:7s}: n={mask.sum():4d}  中位={np.median(vals):.3f}  '
              f'均值={np.mean(vals):.3f}  std={np.std(vals):.3f}')

# ── 写出 CSV ─────────────────────────────────────────────────────────────────
print(f'\n写出: {args.out}')
fieldnames_orig = list(rows[0].keys())
new_fields = ['intensity', 'rho_norm', 'nn_dist_m']

with open(args.out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=fieldnames_orig + new_fields)
    w.writeheader()
    for i, row in enumerate(rows):
        row = dict(row)
        row['intensity']  = f'{intensities[i]:.2f}' if not math.isnan(intensities[i]) else ''
        row['rho_norm']   = f'{rho_norm[i]:.4f}'    if not math.isnan(rho_norm[i])   else ''
        row['nn_dist_m']  = f'{dists[i]:.3f}'
        w.writerow(row)

print(f'完成: {len(rows)} 行，新列 intensity / rho_norm / nn_dist_m')
print(f'\n下一步: python3 lidar_step6c_rho_vs_cn0.py \\')
print(f'          --refl {args.out} \\')
print(f'          --gnss /path/to/training_data.csv')
