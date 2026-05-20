#!/usr/bin/env python3
"""
lidar_step4_ray_casting.py
从 3D 点云地图中，对每颗卫星做射线追踪，判断 LOS / NLOS。

原理：
  - 地图点云（ENU 坐标，urbannav_map.pcd）体素化为 3D 占用网格
  - 对每个历元：
      接收机位置（ENU）→ 根据 epoch_sat_azel.csv 的 azimuth/elevation
      沿该方向发射射线，步进 voxel_size 检查占用
      若在 max_range 内遇到占用体素 → NLOS；否则 → LOS

输入：
  - epoch_sat_azel.csv（Step 3 输出）
  - urbannav_map.pcd（Step 2 输出，ENU 坐标）

输出：
  - lidar_nlos_prediction.csv，列：unix_t, utc_t, sat_id, sys, prn,
    elevation_deg, azimuth_deg, lidar_nlos (0=LOS, 1=NLOS), hit_dist_m

用法：
  python3 lidar_step4_ray_casting.py \
    --azel  /root/epoch_sat_azel.csv \
    --pcd   /root/urbannav_map.pcd \
    --out   /root/lidar_nlos_prediction.csv \
    --voxel 0.5 \
    --max_range 80.0 \
    --rx_height_offset 0.0
"""

import argparse, csv, math, struct, time
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--azel',      default='/root/epoch_sat_azel.csv')
ap.add_argument('--pcd',       default='/root/urbannav_map.pcd')
ap.add_argument('--out',       default='/root/lidar_nlos_prediction.csv')
ap.add_argument('--voxel',     type=float, default=0.5,
                help='体素大小（米），射线步进粒度（默认 0.5m）')
ap.add_argument('--max_range', type=float, default=80.0,
                help='射线最大追踪距离（米），超出则判 LOS（默认 80m）')
ap.add_argument('--traj',      default='/root/novatel_trajectory.csv',
                help='INSPVAX 轨迹 CSV（用于确定 ENU 原点，与 Step 2 一致）')
ap.add_argument('--rx_height_offset', type=float, default=0.0,
                help='接收机高度相对地图的补偿（米），通常为 0')
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

    # 找 x/y/z 列索引
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
            # 解压 lzf
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

        # 每点字节大小
        point_step = sum(sizes)
        # 构建 numpy dtype
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
# 转体素索引
vox_idx = np.floor((cloud - cloud_min) / vs).astype(np.int32)
# 存入 set，用 tuple 或 packed int 加速查询
# 用 uint64 编码 (ix, iy, iz)，各用 21 bit（最大 ~2M 体素/轴）
ix, iy, iz = vox_idx[:, 0], vox_idx[:, 1], vox_idx[:, 2]

# 检查范围
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

def ray_cast(rx_enu, azim_deg, elev_deg, max_range, step):
    """
    从 rx_enu 向 (azim_deg, elev_deg) 方向发射射线。
    返回 (nlos: bool, hit_dist: float)。
    azim: 从正北顺时针，ENU 框架：e=sin(azim)*cos(elev), n=cos(azim)*cos(elev), u=sin(elev)
    """
    az_r  = math.radians(azim_deg)
    el_r  = math.radians(elev_deg)
    cos_el = math.cos(el_r)
    de = math.sin(az_r) * cos_el   # ENU 东分量
    dn = math.cos(az_r) * cos_el   # ENU 北分量
    du = math.sin(el_r)             # ENU 天顶分量

    x0, y0, z0 = rx_enu
    dist = step  # 从一个 step 处开始（跳过接收机自身体素）
    while dist <= max_range:
        px = x0 + de * dist
        py = y0 + dn * dist
        pz = z0 + du * dist
        if is_occupied(px, py, pz):
            return True, dist
        dist += step
    return False, max_range

# ── 读 azel CSV，做射线追踪 ─────────────────────────────────────────
print(f'\n开始射线追踪（max_range={args.max_range}m，step={args.voxel}m）...')
t2 = time.time()

# 需要把接收机位置转到 ENU（Step 3 输出的是 LLH，用与地图相同的 ENU 原点）
# ENU 原点：地图里 cloud_min 只是包围盒原点，不是 ENU 原点。
# Step 3 的 rx_lat/rx_lon/rx_alt 与 Step 2 使用的 INSPVAX 一致。
# 地图坐标系原点 = traj[0]（第一个 INSPVAX 位姿）的 ENU 原点。
# 需从 azel CSV 重建 ENU 坐标：读第一行的 rx_lat/rx_lon/rx_alt 不够，
# 因为我们不知道 ENU 原点，需要记录它。
#
# 解决方案：从 azel CSV 读取 rx_lat/rx_lon/rx_alt，
# 以第一个历元的接收机位置作为 ENU 原点（与 Step 2 对齐）。
# 但 Step 2 用 traj[0] 作为 ENU 原点，此处应使用相同原点。
# → 从 novatel_trajectory.csv 的第一行读 ref_lat/ref_lon/ref_alt。

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

# 读 ENU 原点：必须与 Step 2 完全一致，即 traj[0]（novatel_trajectory.csv 首行）
with open(args.traj) as f:
    traj_row0 = next(csv.DictReader(f))
ref_lat = float(traj_row0['lat'])
ref_lon = float(traj_row0['lon'])
ref_alt = float(traj_row0['alt_m'])
ref_ecef = lla_to_ecef(ref_lat, ref_lon, ref_alt)
print(f'ENU 原点: lat={ref_lat:.5f}, lon={ref_lon:.5f}（来自 novatel_trajectory.csv 首行，与 Step 2 一致）')

COLS_OUT = ['unix_t', 'utc_t', 'sat_id', 'sys', 'prn',
            'elevation_deg', 'azimuth_deg', 'lidar_nlos', 'hit_dist_m']

out_rows = []
nlos_count = 0
los_count  = 0
step = args.voxel  # 射线步进等于体素大小

with open(args.azel) as f:
    reader = csv.DictReader(f)
    all_azel = list(reader)

total = len(all_azel)
for i, row in enumerate(all_azel):
    rx_lat = float(row['rx_lat'])
    rx_lon = float(row['rx_lon'])
    rx_alt = float(row['rx_alt']) + args.rx_height_offset

    # LLH → ECEF → ENU（以 ref 为原点）
    rx_ecef = lla_to_ecef(rx_lat, rx_lon, rx_alt)
    rx_enu  = ecef_to_enu(rx_ecef, ref_ecef, ref_lat, ref_lon)

    azim_deg = float(row['azimuth_deg'])
    elev_deg = float(row['elevation_deg'])

    nlos, hit_dist = ray_cast(rx_enu, azim_deg, elev_deg, args.max_range, step)

    if nlos:
        nlos_count += 1
    else:
        los_count += 1

    out_rows.append({
        'unix_t':       row['unix_t'],
        'utc_t':        row['utc_t'],
        'sat_id':       row['sat_id'],
        'sys':          row['sys'],
        'prn':          row['prn'],
        'elevation_deg': row['elevation_deg'],
        'azimuth_deg':  row['azimuth_deg'],
        'lidar_nlos':   1 if nlos else 0,
        'hit_dist_m':   f'{hit_dist:.2f}',
    })

    if (i + 1) % 500 == 0:
        elapsed = time.time() - t2
        rate = (i + 1) / elapsed
        remain = (total - i - 1) / rate if rate > 0 else 0
        print(f'\r  {i+1}/{total} 条，NLOS={nlos_count} LOS={los_count}，'
              f'速度 {rate:.0f} 条/s，剩余 {remain:.0f}s', end='', flush=True)

print(f'\n射线追踪完成，耗时 {time.time()-t2:.0f}s')
print(f'  LOS: {los_count}，NLOS: {nlos_count}，NLOS 比例: {100*nlos_count/max(total,1):.1f}%')

# ── 写输出 ──────────────────────────────────────────────────────────
with open(args.out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=COLS_OUT)
    w.writeheader()
    w.writerows(out_rows)

print(f'已保存：{args.out}')
print('\n下一步：运行 lidar_step5_compare.py 与 del2AINLOS 标签对比')
