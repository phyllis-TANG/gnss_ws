#!/usr/bin/env python3
"""
make_map.py — 读取 /root/trajectory.csv，生成 /root/trajectory.html
用法：python3 /root/gnss_ws/make_map.py
"""

import csv, os

CSV_PATH  = '/root/trajectory.csv'
HTML_PATH = '/root/trajectory.html'

spp_pts, pvt_pts = [], []

with open(CSV_PATH, 'r') as f:
    for row in csv.DictReader(f):
        pt = [float(row['lat']), float(row['lon'])]
        if row['source'] == 'spp':
            spp_pts.append(pt)
        else:
            pvt_pts.append(pt)

if not spp_pts:
    print('CSV 里没有 SPP 数据，请确认 SPP 节点正常工作')
    exit(1)

clat = sum(p[0] for p in spp_pts) / len(spp_pts)
clon = sum(p[1] for p in spp_pts) / len(spp_pts)

html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"/><title>SPP 轨迹</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body{{height:100%;margin:0}}#map{{height:100%}}
.box{{position:absolute;top:10px;right:10px;z-index:999;background:rgba(255,255,255,.93);
padding:12px 16px;border-radius:8px;font:13px system-ui;box-shadow:0 2px 8px rgba(0,0,0,.2)}}</style>
</head><body>
<div id="map"></div>
<div class="box"><b>GNSS SPP 轨迹</b><br>
<span style="color:#2196F3">■</span> SPP ({len(spp_pts)} pts)<br>
<span style="color:#FF9800">■</span> PVT ref ({len(pvt_pts)} pts)<br>
中心 {clat:.5f}°N, {clon:.5f}°E</div>
<script>
var map=L.map('map').setView([{clat},{clon}],16);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
  {{maxZoom:19,attribution:'© OpenStreetMap'}}).addTo(map);
var s={spp_pts},p={pvt_pts};
if(s.length){{L.polyline(s,{{color:'#2196F3',weight:3}}).addTo(map);
  L.circleMarker(s[0],{{radius:6,color:'#1565C0',fillColor:'#4CAF50',fillOpacity:1}}).bindTooltip('起点').addTo(map);
  L.circleMarker(s[s.length-1],{{radius:6,color:'#1565C0',fillColor:'#F44336',fillOpacity:1}}).bindTooltip('终点').addTo(map);
  map.fitBounds(L.latLngBounds(s).pad(.15));}}
if(p.length) L.polyline(p,{{color:'#FF9800',weight:2,dashArray:'5 4',opacity:.7}}).addTo(map);
</script></body></html>"""

with open(HTML_PATH, 'w') as f:
    f.write(html)

print(f'完成！')
print(f'  SPP: {len(spp_pts)} 点，PVT: {len(pvt_pts)} 点')
print(f'  HTML: {HTML_PATH}')
print(f'  拷出来：sudo docker cp $(sudo docker ps -q):/root/trajectory.html ~/trajectory.html')
