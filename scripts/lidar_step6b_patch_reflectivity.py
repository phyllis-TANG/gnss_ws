#!/usr/bin/env python3
"""
Extract patch-level calibrated LiDAR reflectivity around predicted reflection hits.

The UrbanNav HDL-32E PointCloud2 intensity values behave like calibrated
reflectivity (integer-like 0..255 and nearly range-independent).  Therefore this
script deliberately does NOT multiply intensity by range squared or divide it
by cos(incidence).  Incidence and range remain separate model covariates.

For every reflection hit, neighboring map voxels are collected in a sphere. If
a valid surface normal is available, neighbors are also restricted to a thin
slab around the hit tangent plane.  Patch median reflectivity is the primary
feature; IQR, MAD, point count and nearest distance quantify its uncertainty.

Input PCD is the existing binary x/y/z/intensity map produced by
lidar_step2b_build_map_intensity.py. Its intensity is already a 0.2 m voxel
mean, so the output should be described as patch statistics over voxel-mean
calibrated reflectivity.
"""

import argparse
import csv
import math
import time

import numpy as np


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', default='/root/urbannav_map_intensity.pcd')
    ap.add_argument('--refl', default='/root/lidar_reflection_calibrated_mg_diffuse.csv')
    ap.add_argument('--out', default='/root/lidar_reflection_patch_mg.csv')
    ap.add_argument('--radius', type=float, default=0.5,
                    help='3-D neighborhood radius in metres')
    ap.add_argument('--plane_tol', type=float, default=0.15,
                    help='maximum normal-direction distance from tangent plane')
    ap.add_argument('--min_points', type=int, default=5)
    ap.add_argument('--max_diffuse', type=float, default=100.0,
                    help='HDL-32E diffuse-reflectivity upper limit')
    return ap.parse_args()


def read_binary_xyzi_pcd(path):
    """Read the binary x/y/z/intensity PCD written by step2b."""
    with open(path, 'rb') as f:
        fields = None
        sizes = None
        counts = None
        n_points = None
        data_kind = None
        while True:
            line_b = f.readline()
            if not line_b:
                raise ValueError('PCD header ended before DATA')
            line = line_b.decode('ascii', errors='ignore').strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            key = parts[0].upper()
            if key == 'FIELDS':
                fields = parts[1:]
            elif key == 'SIZE':
                sizes = [int(v) for v in parts[1:]]
            elif key == 'COUNT':
                counts = [int(v) for v in parts[1:]]
            elif key == 'POINTS':
                n_points = int(parts[1])
            elif key == 'DATA':
                data_kind = parts[1].lower()
                break

        if data_kind != 'binary':
            raise ValueError(f'Only binary PCD is supported, got DATA {data_kind}')
        if fields != ['x', 'y', 'z', 'intensity']:
            raise ValueError(f'Expected FIELDS x y z intensity, got {fields}')
        if sizes != [4, 4, 4, 4] or (counts and counts != [1, 1, 1, 1]):
            raise ValueError(f'Unexpected PCD field layout SIZE={sizes} COUNT={counts}')
        if n_points is None:
            raise ValueError('POINTS is missing from PCD header')

        raw = f.read(n_points * 16)
        if len(raw) != n_points * 16:
            raise ValueError(f'Truncated PCD: expected {n_points * 16} bytes, got {len(raw)}')
        return np.frombuffer(raw, dtype=np.float32).reshape(n_points, 4)


def finite_float(row, names):
    for name in names:
        value = row.get(name, '')
        if value in ('', None):
            continue
        try:
            x = float(value)
            if math.isfinite(x):
                return x
        except ValueError:
            pass
    return float('nan')


def row_hit(row):
    return np.array([
        finite_float(row, ['hit_enu_e']),
        finite_float(row, ['hit_enu_n']),
        finite_float(row, ['hit_enu_u']),
    ], dtype=float)


def row_normal(row):
    n = np.array([
        finite_float(row, ['normal_x', 'normal_e']),
        finite_float(row, ['normal_y', 'normal_n']),
        finite_float(row, ['normal_z', 'normal_u']),
    ], dtype=float)
    if not np.all(np.isfinite(n)):
        return None
    norm = np.linalg.norm(n)
    if norm < 1e-6:
        return None
    return n / norm


def main():
    args = parse_args()
    from scipy.spatial import cKDTree

    if args.radius <= 0 or args.plane_tol <= 0 or args.min_points < 1:
        raise SystemExit('radius, plane_tol and min_points must be positive')

    print(f'Reading map: {args.pcd}')
    t0 = time.time()
    cloud = read_binary_xyzi_pcd(args.pcd)
    valid = np.all(np.isfinite(cloud), axis=1)
    valid &= cloud[:, 3] > 0
    cloud = cloud[valid]
    print(f'  valid voxels: {len(cloud):,}; elapsed={time.time()-t0:.1f}s')

    print('Building KD-tree...')
    tree = cKDTree(cloud[:, :3])
    print(f'  elapsed={time.time()-t0:.1f}s')

    with open(args.refl) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        original_fields = list(reader.fieldnames or [])
    if not rows:
        raise SystemExit('Reflection CSV contains no rows')

    stats = []
    n_no_hit = 0
    n_no_patch = 0
    n_plane_used = 0

    for i, row in enumerate(rows):
        hit = row_hit(row)
        if not np.all(np.isfinite(hit)):
            stats.append(None)
            n_no_hit += 1
            continue

        idx = tree.query_ball_point(hit, args.radius)
        if not idx:
            stats.append(None)
            n_no_patch += 1
            continue

        idx = np.asarray(idx, dtype=int)
        pts = cloud[idx, :3]
        intensity = cloud[idx, 3].astype(float)
        delta = pts - hit
        distance = np.linalg.norm(delta, axis=1)

        normal = row_normal(row)
        plane_used = normal is not None
        if plane_used:
            plane_distance = np.abs(delta @ normal)
            keep = plane_distance <= args.plane_tol
            pts = pts[keep]
            intensity = intensity[keep]
            distance = distance[keep]
            n_plane_used += 1

        finite = np.isfinite(intensity) & np.isfinite(distance) & (intensity > 0)
        intensity = intensity[finite]
        distance = distance[finite]
        if len(intensity) == 0:
            stats.append(None)
            n_no_patch += 1
            continue

        retro_fraction = float(np.mean(intensity > args.max_diffuse))
        diffuse = intensity[intensity <= args.max_diffuse]
        diffuse_distance = distance[intensity <= args.max_diffuse]
        if len(diffuse) < args.min_points:
            stats.append(None)
            n_no_patch += 1
            continue

        q25, med, q75 = np.percentile(diffuse, [25, 50, 75])
        mad = np.median(np.abs(diffuse - med))
        stats.append({
            'patch_reflectivity': float(med),
            'patch_q25': float(q25),
            'patch_q75': float(q75),
            'patch_iqr': float(q75 - q25),
            'patch_mad': float(mad),
            'patch_std': float(np.std(diffuse)),
            'patch_n': int(len(diffuse)),
            'patch_nearest_m': float(np.min(diffuse_distance)),
            'patch_retro_fraction': retro_fraction,
            'patch_plane_filter': int(plane_used),
        })

        if (i + 1) % 500 == 0:
            print(f'  processed {i+1}/{len(rows)}')

    medians = np.array([
        s['patch_reflectivity'] for s in stats if s is not None
    ], dtype=float)
    if len(medians) == 0:
        raise SystemExit('No valid patches were extracted')
    global_median = float(np.median(medians))

    added_fields = [
        'rho_norm_legacy_input', 'patch_reflectivity', 'patch_rho_norm',
        'patch_q25', 'patch_q75', 'patch_iqr', 'patch_mad', 'patch_std',
        'patch_n', 'patch_nearest_m', 'patch_retro_fraction',
        'patch_plane_filter', 'patch_radius_m', 'patch_plane_tol_m',
    ]
    out_fields = original_fields + [f for f in added_fields if f not in original_fields]

    written = 0
    with open(args.out, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=out_fields)
        writer.writeheader()
        for row, s in zip(rows, stats):
            if s is None:
                continue
            out = dict(row)
            out['rho_norm_legacy_input'] = row.get('rho_norm', '')
            out['patch_reflectivity'] = f"{s['patch_reflectivity']:.6f}"
            out['patch_rho_norm'] = f"{s['patch_reflectivity']/global_median:.6f}"
            out['rho_norm'] = out['patch_rho_norm']
            out['intensity'] = out['patch_reflectivity']
            out['patch_q25'] = f"{s['patch_q25']:.6f}"
            out['patch_q75'] = f"{s['patch_q75']:.6f}"
            out['patch_iqr'] = f"{s['patch_iqr']:.6f}"
            out['patch_mad'] = f"{s['patch_mad']:.6f}"
            out['patch_std'] = f"{s['patch_std']:.6f}"
            out['patch_n'] = s['patch_n']
            out['patch_nearest_m'] = f"{s['patch_nearest_m']:.6f}"
            out['patch_retro_fraction'] = f"{s['patch_retro_fraction']:.6f}"
            out['patch_plane_filter'] = s['patch_plane_filter']
            out['patch_radius_m'] = args.radius
            out['patch_plane_tol_m'] = args.plane_tol
            writer.writerow(out)
            written += 1

    print('\nPatch extraction summary')
    print(f'  input rows:              {len(rows)}')
    print(f'  output rows:             {written}')
    print(f'  missing hit coordinates: {n_no_hit}')
    print(f'  invalid/small patches:   {n_no_patch}')
    print(f'  plane filter available:  {n_plane_used}')
    print(f'  global patch median:     {global_median:.3f}')
    print(f'  patch n median:          {np.median([s["patch_n"] for s in stats if s]):.0f}')
    print(f'  patch IQR median:        {np.median([s["patch_iqr"] for s in stats if s]):.3f}')
    print(f'  output:                  {args.out}')


if __name__ == '__main__':
    main()
