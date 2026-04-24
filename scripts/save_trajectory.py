#!/usr/bin/env python3
"""
save_trajectory.py
订阅 SPP 定位结果，保存为 CSV + 离线 HTML 地图
用法：在跑 eval_spp.launch 的同时，在另一个终端运行本脚本
结束后按 Ctrl+C，自动生成 /root/gnss_ws/spp_results/trajectory.html
"""

import rospy
import csv
import os
import signal
import sys
from sensor_msgs.msg import NavSatFix

spp_pts = []
pvt_pts = []

OUT_DIR = '/root/gnss_ws/spp_results'

def spp_cb(msg):
    if abs(msg.latitude) < 0.01:
        return
    spp_pts.append((msg.latitude, msg.longitude, msg.altitude))
    print(f'\r[SPP] {len(spp_pts)} pts  lat={msg.latitude:.6f}  lon={msg.longitude:.6f}  alt={msg.altitude:.1f}m', end='', flush=True)

def pvt_cb(msg):
    if abs(msg.latitude) < 0.01:
        return
    pvt_pts.append((msg.latitude, msg.longitude, msg.altitude))

def save_and_exit(sig=None, frame=None):
    print(f'\n\n保存中...')

    # ── CSV ──────────────────────────────────────────────────────────────────
    csv_path = os.path.join(OUT_DIR, 'spp_trajectory.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['lat', 'lon', 'alt_m'])
        w.writerows(spp_pts)
    print(f'CSV 已保存: {csv_path}')

    # ── HTML ─────────────────────────────────────────────────────────────────
    if not spp_pts:
        print('没有收到 SPP 数据，跳过 HTML 生成')
        sys.exit(0)

    center_lat = sum(p[0] for p in spp_pts) / len(spp_pts)
    center_lon = sum(p[1] for p in spp_pts) / len(spp_pts)

    spp_js  = str([[p[0], p[1]] for p in spp_pts])
    pvt_js  = str([[p[0], p[1]] for p in pvt_pts])

    html = f"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8"/>
<title>GNSS SPP 轨迹</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
* {{ margin:0; padding:0; box-sizing:border-box; }}
html, body {{ height:100%; font-family: system-ui, sans-serif; }}
#map {{ height:100%; width:100%; }}
.panel {{
  position:absolute; top:12px; right:12px; z-index:1000;
  background:rgba(255,255,255,0.93); border-radius:10px;
  padding:14px 18px; box-shadow:0 2px 12px rgba(0,0,0,.2);
  font-size:13px; line-height:1.8; min-width:200px;
}}
.dot {{ display:inline-block; width:12px; height:12px; border-radius:50%; margin-right:6px; }}
</style>
</head>
<body>
<div id="map"></div>
<div class="panel">
  <b>GNSS SPP 轨迹</b><br>
  <span class="dot" style="background:#2196F3"></span>SPP ({len(spp_pts)} 点)<br>
  <span class="dot" style="background:#FF9800"></span>u-blox PVT ({len(pvt_pts)} 点)<br>
  <hr style="margin:6px 0">
  中心: {center_lat:.5f}°N<br>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;{center_lon:.5f}°E
</div>
<script>
var spp = {spp_js};
var pvt = {pvt_js};
var map = L.map('map').setView([{center_lat}, {center_lon}], 16);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  maxZoom:19, attribution:'© OpenStreetMap contributors'
}}).addTo(map);
if (spp.length) {{
  L.polyline(spp, {{color:'#2196F3', weight:3}}).addTo(map);
  L.circleMarker(spp[0],  {{radius:6, color:'#1565C0', fillColor:'#2196F3', fillOpacity:1}}).bindTooltip('起点').addTo(map);
  L.circleMarker(spp[spp.length-1], {{radius:6, color:'#1565C0', fillColor:'#00BCD4', fillOpacity:1}}).bindTooltip('终点').addTo(map);
}}
if (pvt.length) {{
  L.polyline(pvt, {{color:'#FF9800', weight:2, dashArray:'6 4', opacity:0.7}}).addTo(map);
}}
if (spp.length) map.fitBounds(L.latLngBounds(spp).pad(0.15));
</script>
</body>
</html>"""

    html_path = os.path.join(OUT_DIR, 'trajectory.html')
    with open(html_path, 'w') as f:
        f.write(html)
    print(f'HTML 已保存: {html_path}')
    print(f'\n把这个文件复制到 Windows，用浏览器打开即可看到地图！')
    sys.exit(0)

signal.signal(signal.SIGINT, save_and_exit)
signal.signal(signal.SIGTERM, save_and_exit)

rospy.init_node('save_trajectory', anonymous=True)
rospy.Subscriber('/gnss_spp_node/spp/navsatfix',   NavSatFix, spp_cb, queue_size=1000)
rospy.Subscriber('/ublox_driver/receiver_lla',      NavSatFix, pvt_cb, queue_size=1000)
print('[save_trajectory] 开始记录，跑完 bag 后按 Ctrl+C 生成地图文件...')
rospy.spin()
