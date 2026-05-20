#!/usr/bin/env python3
"""
lidar_step1_extract_trajectory.py
从 UrbanNav bag 提取 NovAtel INSPVAX 轨迹，保存为 CSV。

用法（在容器 ros1_gnss_lidar2 里）：
  source /root/gnss_ws/devel/setup.bash
  python3 /root/lidar_step1_extract_trajectory.py \
    --bag /bags/UrbanNav-HK_TST-20210517_sensors.bag \
    --out /root/novatel_trajectory.csv
"""

import argparse, csv, math
import rosbag

ap = argparse.ArgumentParser()
ap.add_argument('--bag', default='/bags/UrbanNav-HK_TST-20210517_sensors.bag')
ap.add_argument('--out', default='/root/novatel_trajectory.csv')
args = ap.parse_args()

TOPIC = '/novatel_data/inspvax'

print(f'读取 bag: {args.bag}')
print(f'topic: {TOPIC}')

rows = []
with rosbag.Bag(args.bag, 'r') as bag:
    total = bag.get_message_count(TOPIC)
    print(f'共 {total} 条 INSPVAX 消息，开始提取...')

    for i, (_, msg, t) in enumerate(bag.read_messages(topics=[TOPIC])):
        unix_t = t.to_sec()

        # INSPVAX 字段：lat/lon 单位是度，高度单位是米
        lat  = msg.latitude
        lon  = msg.longitude
        alt  = msg.altitude
        roll    = msg.roll      # 度
        pitch   = msg.pitch     # 度
        heading = msg.azimuth   # 度（NovAtel 用 azimuth 表示航向）

        # 解算状态（SOLUTION_GOOD=3 才可靠）
        ins_status = msg.ins_status if isinstance(msg.ins_status, int) else msg.ins_status.status

        rows.append([
            f'{unix_t:.3f}', lat, lon, alt,
            roll, pitch, heading, ins_status
        ])

        if i % 100 == 0:
            print(f'\r  {i}/{total}  lat={lat:.5f} lon={lon:.5f} alt={alt:.1f}m  status={ins_status}', end='', flush=True)

print(f'\n提取完成，共 {len(rows)} 个位姿')

with open(args.out, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['unix_t', 'lat', 'lon', 'alt_m', 'roll_deg', 'pitch_deg', 'heading_deg', 'ins_status'])
    w.writerows(rows)

print(f'已保存: {args.out}')

# 简单统计
lats = [r[1] for r in rows]
lons = [r[2] for r in rows]
good = sum(1 for r in rows if r[7] == 3)
print(f'\n--- 统计 ---')
print(f'纬度范围:  {min(lats):.5f} ~ {max(lats):.5f}')
print(f'经度范围:  {min(lons):.5f} ~ {max(lons):.5f}')
print(f'INS GOOD 比例: {good}/{len(rows)} ({100*good/len(rows):.1f}%)')
print(f'时间范围: {rows[0][0]} ~ {rows[-1][0]}')
