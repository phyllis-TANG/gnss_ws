#!/usr/bin/env python3
"""
lidar_step4_ray_casting.py
从 3D 点云地图中，对每颗卫星做射线追踪，判断 LOS / NLOS，并支持 2-bounce 多径建模。

原理：
  - 地图点云（ENU 坐标，urbannav_map.pcd）体素化为 3D 占用网格
  - 对每个历元：
      接收机位置（ENU）→ 根据 epoch_sat_azel.csv 的 azimuth/elevation
      沿该方向发射射线，步进 voxel_size 检查占用
      若在 max_range 内遇到占用体素 → NLOS；否则 → LOS
  - 2-bounce：在第一个反射点用 PCA 估计表面法向量，沿镜面反射方向
      继续追踪第二个反射面，输出双反射的原始路径长度

输入：
  - epoch_sat_azel.csv（Step 3 输出）
  - urbannav_map.pcd（Step 2 输出，ENU 坐标）

输出：
  - lidar_nlos_prediction.csv，列：unix_t, utc_t, sat_id, sys, prn,
    elevation_deg, azimuth_deg, lidar_nlos (0=LOS, 1=NLOS), hit_dist_m,
    n_bounces, hit_dist2_m, normal_e, normal_n, normal_u

用法：
  python3 lidar_step4_ray_casting.py \
    --azel  /root/epoch_sat_azel.csv \
    --pcd   /root/urbannav_map.pcd \
    --out   /root/lidar_nlos_prediction.csv \
    --voxel 0.5 \
    --max_range 80.0 \
    --rx_height_offset 0.0 \
    --max_range2 30.0 \
    --normal_radius 1.5
"""

import argparse, csv, math, struct, time
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--azel',      default='/root/epoch_sat_azel.csv')
ap.add_argument('--pcd',       default='/root/urbannav_map.pcd')
ap.add_argument('--out',       default='/root/lidar_nlos_prediction.csv')
ap.add_argument('--voxel',      type=float, default=0.5,
                help='体素大小（米），射线步进粒度（默认 0.5m）')
ap.add_argument('--max_range', type=float, default=80.0,
                help='射线最大追踪距离（米），超出则判 LOS（默认 80m）')
ap.add_argument('--start_dist', type=float, default=5.0,
                help='射线起始距离（米），跳过接收机周围碎点（默认 5m）')
ap.add_argument('--min_z',     type=float, default=1.0,
                help='体素地图最低 Z（ENU，米），过滤地面点（默认 1m，即接收机高度以上）')
ap.add_argument('--traj',      default='/root/novatel_trajectory.csv',
                help='INSPVAX 轨迹 CSV（用于确定 ENU 原点，与 Step 2 一致）')
ap.add_argument('--rx_height_offset', type=float, default=0.0,
                help='接收机高度相对地图的补偿（米），通常为 0')
ap.add_argument('--max_range2', type=float, default=30.0,
                help='2-bounce 第二段射线最大追踪距离（米，默认 30m）')
ap.add_argument('--normal_radius', type=float, default=1.5,
                help='法向量 PCA 搜索半径（米，默认 1.5m）')
args = ap.parse_args()

# ── 读 PCD 文件（ASCII 或 binary） ─────────────────────────────────
print(f'读取点云地图: {args.pcd}')
t0 = time.time()

def read_pcd(path):
    """简单 PCD 读取，支持 ASCII 和 binary 格式，只取 x/y/z。"""
    header = {}
    data_start = 0
    with open(path, 'rb') as f:
        raw = f.read()

    lines = raw.split(b'\n')
    for i, line in enumerate(lines):
        stripped = line.strip().decode('latin-1', errors='replace')
        if stripped.startswith('DATA'):
            header['DATA'] = stripped.split()[1]
            data_start = i + 1
            break
        parts = stripped.split()
        if len(parts) >= 2:
            header[parts[0]] = parts[1:]

    n_pts   = int(header.get('POINTS', [0])[0])
    fields  = header.get('FIELDS', [])
    sizes   = [int(s) for s in header.get('SIZE', [])]
    types   = header.get('TYPE', [])
    data_fmt = header.get('DATA', 'ascii')

    try:
        xi = fields.index('x')
        yi = fields.index('y')
        zi = fields.index('z')
    except ValueError:
        raise RuntimeError('PCD 文件缺少 x/y/z 字段')

    if data_fmt == 'ascii':
        pts = []
        for line in lines[data_start:]:
            parts = line.strip().split()
            if len(parts) < max(xi, yi, zi) + 1:
                continue
            try:
                pts.append([float(parts[xi]), float(parts[yi]), float(parts[zi])])
            except ValueError:
                continue
        return np.array(pts, dtype=np.float32)

    elif data_fmt in ('binary', 'binary_compressed'):
        if data_fmt == 'binary_compressed':
            import ctypes, io
            byte_offset = sum(len(l) + 1 for l in lines[:data_start])
            raw_data = raw[byte_offset:]
            compressed_size, uncompressed_size = struct.unpack_from('<II', raw_data, 0)
            try:
                import lzf
                data_bytes = lzf.decompress(raw_data[8:8+compressed_size], uncompressed_size)
            except ImportError:
                raise RuntimeError('binary_compressed PCD 需要 python-lzf。请改用 open3d 读取。')
        else:
            byte_offset = sum(len(l) + 1 for l in lines[:data_start])
            data_bytes = raw[byte_offset:]

        point_step = sum(sizes)
        fmt_map = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype_list = []
        for i, (f, s, t) in enumerate(zip(fields, sizes, types)):
            np_type = f'{fmt_map.get(t, "u")}{s}'
            dtype_list.append((f, np_type))
        dtype = np.dtype(dtype_list)
        arr = np.frombuffer(data_bytes[:n_pts * point_step], dtype=dtype)
        return np.column_stack([arr['x'].astype(np.float32),
                                arr['y'].astype(np.float32),
                                arr['z'].astype(np.float32)])
    else:
        raise RuntimeError(f'不支持的 PCD data 格式: {data_fmt}')

cloud = read_pcd(args.pcd)
print(f'  读取 {len(cloud):,} 个点，耗时 {time.time()-t0:.1f}s')

# ── 构建体素占用网格 ────────────────────────────────────────────────
print(f'构建体素网格（voxel={args.voxel}m）...')
t1 = time.time()

vs = args.voxel
cloud_min = cloud.min(axis=0)
z_mask = cloud[:, 2] > args.min_z
cloud_filtered = cloud[z_mask]
print(f'  过滤地面点（Z>{args.min_z}m）后剩余 {len(cloud_filtered):,} 个点')
vox_idx = np.floor((cloud_filtered - cloud_min) / vs).astype(np.int32)
ix, iy, iz = vox_idx[:, 0], vox_idx[:, 1], vox_idx[:, 2]

assert ix.max() < (1 << 21), '地图太大，x 超过 21bit 索引范围'
assert iy.max() < (1 << 21), '地图太大，y 超过 21bit 索引范围'
assert iz.max() < (1 << 21), '地图太大，z 超过 21bit 索引范围'

packed = ix.astype(np.uint64) | (iy.astype(np.uint64) << 21) | (iz.astype(np.uint64) << 42)
voxel_set = set(packed.tolist())
print(f'  {len(voxel_set):,} 个占用体素，耗时 {time.time()-t1:.1f}s')

def is_occupied(px, py, pz):
    """查询点 (px,py,pz) 所在体素是否被占用。"""
    ix = int(math.floor((px - cloud_min[0]) / vs))
    iy = int(math.floor((py - cloud_min[1]) / vs))
    iz = int(math.floor((pz - cloud_min[2]) / vs))
    if ix < 0 or iy < 0 or iz < 0:
        return False
    key = ix | (iy << 21) | (iz << 42)
    return key in voxel_set

def get_surface_normal(hit_x, hit_y, hit_z, radius):
    """
    用 PCA 估计 hit 点处的表面法向量。
    在 radius 内收集相邻占用体素中心，取协方差最小特征向量。
    返回归一化法向量，若邻域不足 3 个体素则返回 None。
    """
    r_vox = int(math.ceil(radius / vs))
    hix = int(math.floor((hit_x - cloud_min[0]) / vs))
    hiy = int(math.floor((hit_y - cloud_min[1]) / vs))
    hiz = int(math.floor((hit_z - cloud_min[2]) / vs))

    nearby = []
    for dx in range(-r_vox, r_vox + 1):
        for dy in range(-r_vox, r_vox + 1):
            for dz in range(-r_vox, r_vox + 1):
                nx, ny, nz = hix + dx, hiy + dy, hiz + dz
                if nx < 0 or ny < 0 or nz < 0:
                    continue
                key = int(nx) | (int(ny) << 21) | (int(nz) << 42)
                if key in voxel_set:
                    cx = (nx + 0.5) * vs + cloud_min[0]
                    cy = (ny + 0.5) * vs + cloud_min[1]
                    cz = (nz + 0.5) * vs + cloud_min[2]
                    nearby.append([cx, cy, cz])

    if len(nearby) < 3:
        return None

    pts = np.array(nearby, dtype=np.float64)
    centroid = pts.mean(axis=0)
    cov = np.cov((pts - centroid).T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    # smallest eigenvalue → normal (direction of least spread = perpendicular to surface)
    return eigenvectors[:, 0]

def ray_cast_from_point(origin, direction, max_range, step):
    """
    从 origin 沿 direction 发射射线（已归一化）。
    跳过起始一步，避免立即重命中出发体素。
    返回 (nlos: bool, hit_dist: float)。
    """
    ox, oy, oz = origin
    dx, dy, dz = direction
    dist = step
    while dist <= max_range:
        px = ox + dx * dist
        py = oy + dy * dist
        pz = oz + dz * dist
        if is_occupied(px, py, pz):
            return True, dist
        dist += step
    return False, max_range

def ray_cast(rx_enu, azim_deg, elev_deg, max_range, step, start_dist):
    """
    从 rx_enu 向 (azim_deg, elev_deg) 方向发射射线。
    返回 (nlos: bool, hit_dist: float)。
    azim: 从正北顺时针，ENU 框架：e=sin(azim)*cos(elev), n=cos(azim)*cos(elev), u=sin(elev)
    """
    az_r  = math.radians(azim_deg)
    el_r  = math.radians(elev_deg)
    cos_el = math.cos(el_r)
    de = math.sin(az_r) * cos_el
    dn = math.cos(az_r) * cos_el
    du = math.sin(el_r)

    x0, y0, z0 = rx_enu
    dist = start_dist
    while dist <= max_range:
        px = x0 + de * dist
        py = y0 + dn * dist
        pz = z0 + du * dist
        if is_occupied(px, py, pz):
            return True, dist
        dist += step
    return False, max_range

# ── 读 azel CSV，做射线追踪 ─────────────────────────────────────────
print(f'\n开始射线追踪（max_range={args.max_range}m，step={args.voxel}m，'
      f'max_range2={args.max_range2}m，normal_radius={args.normal_radius}m）...')
t2 = time.time()

def lla_to_ecef(lat_deg, lon_deg, alt_m):
    a, e2 = 6378137.0, 6.69437999014e-3
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return np.array([x, y, z])

def ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon):
    lat = math.radians(ref_lat)
    lon = math.radians(ref_lon)
    R = np.array([
        [-math.sin(lon),                math.cos(lon),               0],
        [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
        [ math.cos(lat)*math.cos(lon),  math.cos(lat)*math.sin(lon), math.sin(lat)]
    ])
    return R @ (ecef - ref_ecef)

# ENU 原点：与 Step 2 完全一致，即 traj[0]（novatel_trajectory.csv 首行）
with open(args.traj) as f:
    traj_row0 = next(csv.DictReader(f))
ref_lat = float(traj_row0['lat'])
ref_lon = float(traj_row0['lon'])
ref_alt = float(traj_row0['alt_m'])
ref_ecef = lla_to_ecef(ref_lat, ref_lon, ref_alt)
print(f'ENU 原点: lat={ref_lat:.5f}, lon={ref_lon:.5f}（来自 novatel_trajectory.csv 首行，与 Step 2 一致）')

COLS_OUT = ['unix_t', 'utc_t', 'sat_id', 'sys', 'prn',
            'elevation_deg', 'azimuth_deg', 'lidar_nlos', 'hit_dist_m',
            'n_bounces', 'hit_dist2_m', 'normal_e', 'normal_n', 'normal_u']

out_rows = []
nlos_count = 0
los_count  = 0
bounce2_count = 0
step = args.voxel

with open(args.azel) as f:
    reader = csv.DictReader(f)
    all_azel = list(reader)

total = len(all_azel)
for i, row in enumerate(all_azel):
    rx_lat = float(row['rx_lat'])
    rx_lon = float(row['rx_lon'])
    rx_alt = float(row['rx_alt']) + args.rx_height_offset

    rx_ecef = lla_to_ecef(rx_lat, rx_lon, rx_alt)
    rx_enu  = ecef_to_enu(rx_ecef, ref_ecef, ref_lat, ref_lon)

    azim_deg = float(row['azimuth_deg'])
    elev_deg = float(row['elevation_deg'])

    # ── 第一段射线 ────────────────────────────────────────────────────
    nlos, hit_dist = ray_cast(rx_enu, azim_deg, elev_deg, args.max_range, step, args.start_dist)

    n_bounces  = 0
    hit_dist2  = 0.0
    normal_e   = 0.0
    normal_n   = 0.0
    normal_u   = 0.0

    if nlos:
        nlos_count += 1
        n_bounces = 1

        # 计算第一击中点（ENU）
        az_r   = math.radians(azim_deg)
        el_r   = math.radians(elev_deg)
        cos_el = math.cos(el_r)
        de = math.sin(az_r) * cos_el
        dn = math.cos(az_r) * cos_el
        du = math.sin(el_r)
        d  = np.array([de, dn, du])   # 朝卫星方向单位向量

        x0, y0, z0 = rx_enu
        h1x = x0 + de * hit_dist
        h1y = y0 + dn * hit_dist
        h1z = z0 + du * hit_dist

        # ── 估计第一击中面的法向量（PCA）────────────────────────────
        normal = get_surface_normal(h1x, h1y, h1z, args.normal_radius)
        if normal is not None:
            # 确保法向量朝向接收机（与入射方向相反）
            if np.dot(normal, d) > 0:
                normal = -normal
            normal_e, normal_n, normal_u = float(normal[0]), float(normal[1]), float(normal[2])

            # ── 镜面反射方向：d_r = d - 2*(d·n)*n ────────────────
            # 这是从第一反射面看，信号来自哪个方向（用于寻找第二反射面）
            d_dot_n = float(np.dot(d, normal))
            d_refl  = d - 2.0 * d_dot_n * normal
            d_refl_len = float(np.linalg.norm(d_refl))
            if d_refl_len > 1e-6:
                d_refl = d_refl / d_refl_len

                # ── 第二段射线（从第一击中点沿反射方向）────────────
                nlos2, dist2 = ray_cast_from_point(
                    (h1x, h1y, h1z), d_refl,
                    args.max_range2, step
                )
                if nlos2:
                    n_bounces = 2
                    hit_dist2 = dist2
                    bounce2_count += 1
    else:
        los_count += 1

    out_rows.append({
        'unix_t':        row['unix_t'],
        'utc_t':         row['utc_t'],
        'sat_id':        row['sat_id'],
        'sys':           row['sys'],
        'prn':           row['prn'],
        'elevation_deg': row['elevation_deg'],
        'azimuth_deg':   row['azimuth_deg'],
        'lidar_nlos':    1 if nlos else 0,
        'hit_dist_m':    f'{hit_dist:.2f}',
        'n_bounces':     n_bounces,
        'hit_dist2_m':   f'{hit_dist2:.2f}',
        'normal_e':      f'{normal_e:.4f}',
        'normal_n':      f'{normal_n:.4f}',
        'normal_u':      f'{normal_u:.4f}',
    })

    if (i + 1) % 500 == 0:
        elapsed = time.time() - t2
        rate = (i + 1) / elapsed
        remain = (total - i - 1) / rate if rate > 0 else 0
        print(f'\r  {i+1}/{total} 条，NLOS={nlos_count}（2-bounce={bounce2_count}）LOS={los_count}，'
              f'速度 {rate:.0f} 条/s，剩余 {remain:.0f}s', end='', flush=True)

print(f'\n射线追踪完成，耗时 {time.time()-t2:.0f}s')
print(f'  LOS: {los_count}，NLOS: {nlos_count}，NLOS 比例: {100*nlos_count/max(total,1):.1f}%')
print(f'  其中 2-bounce: {bounce2_count}（占 NLOS {100*bounce2_count/max(nlos_count,1):.1f}%）')

# ── 写输出 ──────────────────────────────────────────────────────────
with open(args.out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS_OUT)
    w.writeheader()
    w.writerows(out_rows)

print(f'已保存：{args.out}')
print('\n新增列：n_bounces（0/1/2）、hit_dist2_m（第二击中距离）、normal_e/n/u（第一击中面法向量）')
print('下一步：运行 lidar_step5_compare.py 与 del2AINLOS 标签对比')
