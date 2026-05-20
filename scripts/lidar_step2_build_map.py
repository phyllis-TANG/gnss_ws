#!/usr/bin/env python3
"""
lidar_step2_build_map.py
用 NovAtel INSPVAX 位姿 + Velodyne 点云建立全局 3D 地图，保存为 .pcd 文件。

外参（来自 UrbanNav-HK-Medium-Urban-1/extrinsic.yaml）：
  body 坐标系：x=右, y=前, z=上
  CENTER_LiDAR_T_IMU:  translation (0, 0, +0.28m)，旋转=单位矩阵
  ANTENNA_T_IMU(SPAN): translation (0, 0, +0.14m)，旋转=单位矩阵
  → 雷达相对 SPAN 天线（INSPVAX 参考点）：(0, 0, +0.14m)

用法（容器 ros1_gnss_lidar2）：
  source /root/gnss_ws/devel/setup.bash
  python3 /root/lidar_step2_build_map.py \
    --bag   /bags/UrbanNav-HK_TST-20210517_sensors.bag \
    --traj  /root/novatel_trajectory.csv \
    --out   /root/urbannav_map.pcd \
    --step  10
"""

import argparse, csv, math, time
import numpy as np
import rosbag
from sensor_msgs import point_cloud2

ap = argparse.ArgumentParser()
ap.add_argument('--bag',  default='/bags/UrbanNav-HK_TST-20210517_sensors.bag')
ap.add_argument('--traj', default='/root/novatel_trajectory.csv')
ap.add_argument('--out',  default='/root/urbannav_map.pcd')
ap.add_argument('--step', type=int, default=10,
                help='每隔几帧取一帧（默认10，约785帧→79帧，快速测试）')
ap.add_argument('--max_range', type=float, default=50.0,
                help='最大点云距离（米），过滤远处噪点')
args = ap.parse_args()

LIDAR_TOPIC = '/velodyne_points'
# 雷达相对 SPAN 天线的偏移（body 框架，单位米）
T_LIDAR_FROM_SPAN = np.array([0.0, 0.0, 0.14])

# ── 时间转换工具 ──────────────────────────────────────────────────────────────
def lla_to_ecef(lat_deg, lon_deg, alt_m):
    """WGS84 经纬高 → ECEF"""
    a, e2 = 6378137.0, 6.69437999014e-3
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return np.array([x, y, z])

def ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon):
    """ECEF → 以参考点为原点的 ENU"""
    lat, lon = math.radians(ref_lat), math.radians(ref_lon)
    R = np.array([
        [-math.sin(lon),               math.cos(lon),              0],
        [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
        [ math.cos(lat)*math.cos(lon),  math.cos(lat)*math.sin(lon), math.sin(lat)]
    ])
    return R @ (ecef - ref_ecef)

def rpy_to_rot(roll_deg, pitch_deg, heading_deg):
    """Roll/Pitch/Heading(航向，从北顺时针) → body→ENU 旋转矩阵"""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    # NovAtel azimuth: 从正北顺时针，ENU yaw 从正东逆时针
    y = math.radians(90.0 - heading_deg)

    # ZYX 顺序：先 yaw，再 pitch，再 roll
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

# ── 读轨迹 CSV ────────────────────────────────────────────────────────────────
print('读取轨迹...')
traj = []   # [(unix_t, lat, lon, alt, roll, pitch, heading), ...]
with open(args.traj) as f:
    for row in csv.DictReader(f):
        traj.append((
            float(row['unix_t']),
            float(row['lat']), float(row['lon']), float(row['alt_m']),
            float(row['roll_deg']), float(row['pitch_deg']), float(row['heading_deg'])
        ))
traj.sort(key=lambda x: x[0])
traj_times = np.array([r[0] for r in traj])
print(f'  {len(traj)} 个位姿，时间 {traj[0][0]:.1f} ~ {traj[-1][0]:.1f}')

# 以第一个位置为 ENU 原点
ref_lat, ref_lon = traj[0][1], traj[0][2]
ref_ecef = lla_to_ecef(ref_lat, ref_lon, traj[0][3])
print(f'  ENU 原点: lat={ref_lat:.5f}, lon={ref_lon:.5f}')

def get_pose_at(unix_t):
    """最近邻插值，返回 (enu_pos, R_body2enu)"""
    idx = np.searchsorted(traj_times, unix_t)
    idx = min(max(idx, 0), len(traj) - 1)
    _, lat, lon, alt, roll, pitch, heading = traj[idx]
    ecef = lla_to_ecef(lat, lon, alt)
    enu  = ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon)
    R    = rpy_to_rot(roll, pitch, heading)
    return enu, R

# ── 遍历 bag，累积点云 ────────────────────────────────────────────────────────
print(f'\n开始建图（每 {args.step} 帧取1帧，最大距离 {args.max_range}m）...')
all_points = []
frame_count = 0
t0 = time.time()

with rosbag.Bag(args.bag, 'r') as bag:
    total_lidar = bag.get_message_count(LIDAR_TOPIC)
    print(f'  bag 共 {total_lidar} 帧 LiDAR')

    for i, (_, msg, t) in enumerate(bag.read_messages(topics=[LIDAR_TOPIC])):
        if i % args.step != 0:
            continue

        unix_t = t.to_sec()
        enu_span, R = get_pose_at(unix_t)

        # 雷达在 ENU 的位置 = SPAN 天线位置 + R * T_LIDAR_FROM_SPAN
        enu_lidar = enu_span + R @ T_LIDAR_FROM_SPAN

        # 解析点云
        pts = list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        if not pts:
            continue
        pts_arr = np.array(pts, dtype=np.float32)  # (N, 3) in LiDAR frame

        # 过滤：去掉太远和原点附近（自身反射）
        dist = np.linalg.norm(pts_arr, axis=1)
        mask = (dist < args.max_range) & (dist > 0.5)
        pts_arr = pts_arr[mask]
        if len(pts_arr) == 0:
            continue

        # 变换到 ENU：p_enu = R @ p_lidar + enu_lidar
        pts_world = (R @ pts_arr.T).T + enu_lidar
        all_points.append(pts_world)

        frame_count += 1
        if frame_count % 10 == 0:
            elapsed = time.time() - t0
            print(f'\r  已处理 {frame_count} 帧（{i}/{total_lidar}），'
                  f'累积 {sum(len(p) for p in all_points):,} 点，'
                  f'耗时 {elapsed:.0f}s', end='', flush=True)

print(f'\n合并点云...')
if not all_points:
    print('错误：没有点云数据！')
    exit(1)

cloud = np.vstack(all_points).astype(np.float32)
print(f'总点数: {len(cloud):,}')

# 体素降采样（0.2m），减小文件体积
print('体素降采样（0.2m）...')
try:
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd_down = pcd.voxel_down_sample(voxel_size=0.2)
    o3d.io.write_point_cloud(args.out, pcd_down)
    print(f'降采样后: {len(pcd_down.points):,} 点')
except ImportError:
    # open3d 不可用时直接写原始 PCD
    _write_pcd(args.out, cloud)

print(f'\n地图已保存: {args.out}')
print(f'总耗时: {time.time()-t0:.0f}s')
print('\n下一步：用 CloudCompare 或 rviz 打开 .pcd 文件查看地图质量')
