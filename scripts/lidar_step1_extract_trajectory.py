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

import argparse, csv
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

        # 第一条消息：打印所有可用字段名，方便调试
        if i == 0:
            print(f'\n[调试] INSPVAX 字段列表: {[f for f in dir(msg) if not f.startswith("_")]}')

        unix_t = t.to_sec()
        lat  = msg.latitude
        lon  = msg.longitude
        alt  = msg.altitude
        roll    = msg.roll
        pitch   = msg.pitch
        heading = msg.azimuth   # NovAtel 用 azimuth 表示航向角

        # ins_status 可能是 int 也可能是含 .status 属性的对象
        raw_status = msg.ins_status
        ins_status = raw_status if isinstance(raw_status, int) else int(raw_status.status)

        rows.append([
            f'{unix_t:.3f}', lat, lon, alt,
            roll, pitch, heading, ins_status
        ])

        if i % 100 == 0:
            print(f'\r  {i}/{total}  lat={lat:.5f} lon={lon:.5f} alt={alt:.1f}m  ins={ins_status}', end='', flush=True)

print(f'\n提取完成，共 {len(rows)} 个位姿')

with open(args.out, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['unix_t', 'lat', 'lon', 'alt_m', 'roll_deg', 'pitch_deg', 'heading_deg', 'ins_status'])
    w.writerows(rows)

print(f'已保存: {args.out}')

lats = [r[1] for r in rows]
lons = [r[2] for r in rows]
good = sum(1 for r in rows if r[7] == 3)
print(f'\n--- 统计 ---')
print(f'纬度范围:  {min(lats):.5f} ~ {max(lats):.5f}')
print(f'经度范围:  {min(lons):.5f} ~ {max(lons):.5f}')
print(f'INS GOOD(3) 比例: {good}/{len(rows)} ({100*good/max(len(rows),1):.1f}%)')
print(f'时间范围: {rows[0][0]} ~ {rows[-1][0]}')
