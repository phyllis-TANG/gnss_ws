#!/usr/bin/env python3
"""
save_trajectory.py
每收到一个 SPP 定位点就立刻追加写入 /root/trajectory.csv
脚本随时可以 Ctrl+C，文件里已有的数据不会丢失
结束后运行 python3 /root/generate_analysis.py [--gt <ground_truth_file>] 生成分析报告
"""

import rospy, csv, os
from sensor_msgs.msg import NavSatFix

CSV_PATH = '/root/trajectory.csv'

with open(CSV_PATH, 'w', newline='') as f:
    csv.writer(f).writerow(['timestamp', 'lat', 'lon', 'alt_m', 'source'])

count = 0

def spp_cb(msg):
    global count
    if abs(msg.latitude) < 0.1:
        return
    ts = msg.header.stamp.to_sec()
    with open(CSV_PATH, 'a', newline='') as f:
        csv.writer(f).writerow([f'{ts:.3f}', msg.latitude, msg.longitude, msg.altitude, 'spp'])
    count += 1
    print(f'\r[SPP] {count} pts  {msg.latitude:.5f},{msg.longitude:.5f}', end='', flush=True)

rospy.init_node('save_trajectory', anonymous=True)
rospy.Subscriber('/gnss_spp_node/spp/navsatfix', NavSatFix, spp_cb, queue_size=2000)
print(f'记录中，数据实时写入 {CSV_PATH}')
print('bag 播完后直接 Ctrl+C，然后运行:')
print('  python3 /root/generate_analysis.py                          # 仅 SPP 轨迹')
print('  python3 /root/generate_analysis.py --gt <ground_truth.txt>  # SPP vs 地面真值')
rospy.spin()
