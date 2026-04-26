#!/usr/bin/env python3
"""
save_trajectory.py — 收集 SPP 定位结果，bag 播完后自动生成 HTML 地图
保存位置：/root/trajectory.html
"""

import rospy, os, csv
from sensor_msgs.msg import NavSatFix

spp_pts = []
pvt_pts = []

def spp_cb(msg):
    if abs(msg.latitude) < 0.1:
        return
    spp_pts.append((msg.latitude, msg.longitude, msg.altitude))
    print(f'\r[SPP] {len(spp_pts)} pts  {msg.latitude:.5f},{msg.longitude:.5f}', end='', flush=True)

def pvt_cb(msg):
    if abs(msg.latitude) < 0.1:
        return
    pvt_pts.append((msg.latitude, msg.longitude, msg.altitude))

def on_shutdown():
    print('\n\n--- 生成文件中 ---')

    if not spp_pts:
        print('没有收到 SPP 数据，请确认 SPP 节点在正常输出定位结果')
        return

    # CSV
    csv_path = '/root/trajectory.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['lat', 'lon', 'alt_m'])
        w.writerows(spp_pts)
    print(f'CSV: {csv_path}  ({len(spp_pts)} 行)')

    # HTML
    clat = sum(p[0] for p in spp_pts) / len(spp_pts)
    clon = sum(p[1] for p in spp_pts) / len(spp_pts)
    spp_js = str([[round(p[0],6), round(p[1],6)] for p in spp_pts])
    pvt_js = str([[round(p[0],6), round(p[1],6)] for p in pvt_pts])

    html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"/><title>SPP 轨迹</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body{{height:100%;margin:0}}#map{{height:100%}}.box{{position:absolute;top:10px;right:10px;z-index:999;background:rgba(255,255,255,.9);padding:12px 16px;border-radius:8px;font:13px system-ui;box-shadow:0 2px 8px rgba(0,0,0,.2)}}</style>
</head><body>
<div id="map"></div>
<div class="box"><b>GNSS SPP 轨迹</b><br>
<span style="color:#2196F3">■</span> SPP ({len(spp_pts)} pts)<br>
<span style="color:#FF9800">■</span> PVT ref ({len(pvt_pts)} pts)<br>
中心 {clat:.5f}°N, {clon:.5f}°E</div>
<script>
var map=L.map('map').setView([{clat},{clon}],16);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{maxZoom:19,attribution:'© OSM'}}).addTo(map);
var s={spp_js}, p={pvt_js};
if(s.length){{L.polyline(s,{{color:'#2196F3',weight:3}}).addTo(map);map.fitBounds(L.latLngBounds(s).pad(.15));}}
if(p.length) L.polyline(p,{{color:'#FF9800',weight:2,dashArray:'5 4',opacity:.7}}).addTo(map);
</script></body></html>"""

    html_path = '/root/trajectory.html'
    with open(html_path, 'w') as f:
        f.write(html)
    print(f'HTML: {html_path}')
    print('\n完成！用这条命令拷出来：')
    print('  sudo docker cp $(sudo docker ps -q):/root/trajectory.html ~/trajectory.html')

rospy.init_node('save_trajectory', anonymous=True)
rospy.on_shutdown(on_shutdown)
rospy.Subscriber('/gnss_spp_node/spp/navsatfix', NavSatFix, spp_cb, queue_size=2000)
rospy.Subscriber('/ublox_driver/receiver_lla',   NavSatFix, pvt_cb, queue_size=2000)
print('记录中... bag 播完或按 Ctrl+C 后自动保存')
rospy.spin()
