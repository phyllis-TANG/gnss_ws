#!/usr/bin/env python3
"""
lidar_step2b_build_map_intensity.py
在 lidar_step2_build_map.py 基础上保留 LiDAR intensity 字段，
输出 4 列（x y z intensity）的 ENU 点云，用于反射率衰减模型研究。

外参（UrbanNav-HK-Medium-Urban-1/extrinsic.yaml）：
  CENTER_LiDAR_T_IMU:  translation (0, 0, +0.28m)，旋转=单位矩阵
  ANTENNA_T_IMU(SPAN): translation (0, 0, +0.14m)，旋转=单位矩阵
  → 雷达相对 SPAN 天线偏移: (0, 0, +0.14m)

用法（容器 ros1_gnss）：
  source /root/gnss_ws/devel/setup.bash
  python3 /root/gnss_ws/scripts/lidar_step2b_build_map_intensity.py \
    --bag  /bags/UrbanNav-HK_TST-20210517_sensors.bag \
    --traj /root/novatel_trajectory.csv \
    --out  /root/urbannav_map_intensity.pcd \
    --step 10
"""

import argparse, csv, math, struct, time
import numpy as np
import rosbag
from sensor_msgs import point_cloud2

ap = argparse.ArgumentParser()
ap.add_argument('--bag',       default='/bags/UrbanNav-HK_TST-20210517_sensors.bag')
ap.add_argument('--traj',      default='/root/novatel_trajectory.csv')
ap.add_argument('--out',       default='/root/urbannav_map_intensity.pcd')
ap.add_argument('--step',      type=int,   default=10)
ap.add_argument('--max_range', type=float, default=50.0)
ap.add_argument('--voxel',     type=float, default=0.2,
                help='体素降采样边长（米）；0 = 不降采样')
args = ap.parse_args()

LIDAR_TOPIC = '/velodyne_points'
T_LIDAR_FROM_SPAN = np.array([0.0, 0.0, 0.14])

# ── 坐标工具 ──────────────────────────────────────────────────────────────────
def lla_to_ecef(lat_deg, lon_deg, alt_m):
    a, e2 = 6378137.0, 6.69437999014e-3
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    return np.array([
        (N + alt_m) * math.cos(lat) * math.cos(lon),
        (N + alt_m) * math.cos(lat) * math.sin(lon),
        (N * (1 - e2) + alt_m) * math.sin(lat)
    ])

def ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon):
    lat, lon = math.radians(ref_lat), math.radians(ref_lon)
    R = np.array([
        [-math.sin(lon),                math.cos(lon),               0],
        [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
        [ math.cos(lat)*math.cos(lon),  math.cos(lat)*math.sin(lon), math.sin(lat)]
    ])
    return R @ (ecef - ref_ecef)

def rpy_to_rot(roll_deg, pitch_deg, heading_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(90.0 - heading_deg)
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,            1]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0,           0          ],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    return Rz @ Ry @ Rx

# ── 读轨迹 ────────────────────────────────────────────────────────────────────
print('读取轨迹...')
traj = []
with open(args.traj) as f:
    for row in csv.DictReader(f):
        traj.append((float(row['unix_t']),
                     float(row['lat']), float(row['lon']), float(row['alt_m']),
                     float(row['roll_deg']), float(row['pitch_deg']), float(row['heading_deg'])))
traj.sort(key=lambda x: x[0])
traj_times = np.array([r[0] for r in traj])

ref_lat, ref_lon = traj[0][1], traj[0][2]
ref_ecef = lla_to_ecef(ref_lat, ref_lon, traj[0][3])
print(f'  ENU 原点: lat={ref_lat:.6f}, lon={ref_lon:.6f}  ({len(traj)} 个位姿)')

def _interp_angle(a0, a1, f):
    d = ((a1 - a0 + 180.0) % 360.0) - 180.0
    return a0 + f * d

def get_pose_at(unix_t):
    idx = np.searchsorted(traj_times, unix_t)
    if idx <= 0:   i0 = i1 = 0; f = 0.0
    elif idx >= len(traj): i0 = i1 = len(traj)-1; f = 0.0
    else:
        i0, i1 = idx-1, idx
        t0, t1 = traj_times[i0], traj_times[i1]
        f = (unix_t - t0) / (t1 - t0) if t1 > t0 else 0.0
    _, lat0, lon0, alt0, r0, p0, h0 = traj[i0]
    _, lat1, lon1, alt1, r1, p1, h1 = traj[i1]
    ecef = lla_to_ecef(lat0, lon0, alt0) + f * (lla_to_ecef(lat1, lon1, alt1) - lla_to_ecef(lat0, lon0, alt0))
    enu  = ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon)
    R    = rpy_to_rot(_interp_angle(r0,r1,f), _interp_angle(p0,p1,f), _interp_angle(h0,h1,f))
    return enu, R

# ── 体素降采样（保留 intensity 均值）────────────────────────────────────────────
def voxel_downsample(cloud_xyzI, voxel_size):
    if voxel_size <= 0:
        return cloud_xyzI
    coords = np.floor(cloud_xyzI[:, :3] / voxel_size).astype(np.int32)
    # 哈希：把三维 voxel 坐标合并成一个整数键
    keys = coords[:, 0].astype(np.int64) * 1_000_003 + \
           coords[:, 1].astype(np.int64) * 1_003 + \
           coords[:, 2].astype(np.int64)
    order = np.argsort(keys)
    keys_sorted = keys[order]
    cloud_sorted = cloud_xyzI[order]
    # 找每个 voxel 的起止索引
    splits = np.where(np.diff(keys_sorted))[0] + 1
    starts = np.concatenate([[0], splits])
    ends   = np.concatenate([splits, [len(keys_sorted)]])
    result = []
    for s, e in zip(starts, ends):
        result.append(cloud_sorted[s:e].mean(axis=0))
    return np.array(result, dtype=np.float32)

# ── 写 PCD（x y z intensity，二进制）────────────────────────────────────────────
def write_pcd_intensity(filename, cloud_xyzI):
    n = len(cloud_xyzI)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(filename, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(cloud_xyzI.astype(np.float32).tobytes())
    print(f'已保存: {filename}  ({n:,} 点, 4 列: x y z intensity)')

# ── 遍历 bag ──────────────────────────────────────────────────────────────────
print(f'\n建图（step={args.step}, max_range={args.max_range}m）...')
all_points = []
frame_count = 0
t0 = time.time()

with rosbag.Bag(args.bag, 'r') as bag:
    total_lidar = bag.get_message_count(LIDAR_TOPIC)
    print(f'  bag 共 {total_lidar} 帧 LiDAR')

    for i, (_, msg, t) in enumerate(bag.read_messages(topics=[LIDAR_TOPIC])):
        if i % args.step != 0:
            continue

        unix_t    = t.to_sec()
        enu_span, R = get_pose_at(unix_t)
        enu_lidar = enu_span + R @ T_LIDAR_FROM_SPAN

        # ── 关键改动：读取 intensity 字段 ──────────────────────────────────────
        pts = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
        if not pts:
            continue
        pts_arr = np.array(pts, dtype=np.float32)   # (N, 4): x y z intensity

        # 距离过滤（只用 xyz）
        dist = np.linalg.norm(pts_arr[:, :3], axis=1)
        mask = (dist < args.max_range) & (dist > 0.5)
        pts_arr = pts_arr[mask]
        if len(pts_arr) == 0:
            continue

        # 变换：只旋转/平移 xyz，intensity 原样保留
        xyz_world = (R @ pts_arr[:, :3].T).T + enu_lidar
        pts_world = np.hstack([xyz_world, pts_arr[:, 3:4]])   # (N, 4)
        all_points.append(pts_world)

        frame_count += 1
        if frame_count % 10 == 0:
            elapsed = time.time() - t0
            total_pts = sum(len(p) for p in all_points)
            print(f'\r  {frame_count} 帧 ({i}/{total_lidar}), '
                  f'{total_pts:,} 点, {elapsed:.0f}s', end='', flush=True)

print(f'\n合并点云...')
if not all_points:
    print('错误：无点云数据')
    exit(1)

cloud = np.vstack(all_points).astype(np.float32)
print(f'合并后: {len(cloud):,} 点  (x y z intensity)')
print(f'intensity 范围: {cloud[:,3].min():.1f} ~ {cloud[:,3].max():.1f}')

if args.voxel > 0:
    print(f'体素降采样 ({args.voxel}m)...')
    cloud = voxel_downsample(cloud, args.voxel)
    print(f'降采样后: {len(cloud):,} 点')

write_pcd_intensity(args.out, cloud)
print(f'总耗时: {time.time()-t0:.0f}s')
print('\n下一步: python3 lidar_step6b_extract_intensity.py --pcd', args.out)
