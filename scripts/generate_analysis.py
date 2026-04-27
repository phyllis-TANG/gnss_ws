#!/usr/bin/env python3
"""
generate_analysis.py
读取 /root/trajectory.csv，生成 SPP vs PVT 分析 HTML
用法：python3 /root/generate_analysis.py
"""

import csv, json, math, os

CSV_PATH  = '/root/trajectory.csv'
HTML_PATH = '/root/spp_analysis.html'

# ── 读数据 ────────────────────────────────────────────────────────────────────
spp_pts, pvt_pts = [], []
with open(CSV_PATH) as f:
    for row in csv.DictReader(f):
        pt = [float(row['lat']), float(row['lon'])]
        if row['source'] == 'spp':
            spp_pts.append(pt)
        else:
            pvt_pts.append(pt)

if not spp_pts:
    print('没有 SPP 数据'); exit(1)

# ── 计算统计 ───────────────────────────────────────────────────────────────────
def haversine(p1, p2):
    R = 6371000
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    a = math.sin((lat2-lat1)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
    return 2 * R * math.asin(math.sqrt(a))

# SPP 散布（相邻点距离）
scatter = [haversine(spp_pts[i], spp_pts[i+1]) for i in range(len(spp_pts)-1)]

# SPP vs PVT 逐点距离（按索引配对，时间近似）
n_pair = min(len(spp_pts), len(pvt_pts))
spp_pvt_dists = [haversine(spp_pts[i], pvt_pts[i]) for i in range(n_pair)]

def stats(vals):
    if not vals:
        return dict(mean=0, median=0, rms=0, p95=0, mx=0)
    s = sorted(vals)
    return dict(
        mean   = sum(vals)/len(vals),
        median = s[len(s)//2],
        rms    = math.sqrt(sum(v**2 for v in vals)/len(vals)),
        p95    = s[int(len(s)*0.95)],
        mx     = max(vals),
    )

st = stats(spp_pvt_dists)
clat = sum(p[0] for p in spp_pts) / len(spp_pts)
clon = sum(p[1] for p in spp_pts) / len(spp_pts)

# 时序误差图数据（每 10 点采样一次，避免图太密）
step = max(1, n_pair // 200)
err_series = [{"t": i, "d": round(spp_pvt_dists[i], 2)}
              for i in range(0, n_pair, step)]

# ── 嵌入数据 ───────────────────────────────────────────────────────────────────
DATA = json.dumps({
    "spp": spp_pts,
    "pvt": pvt_pts,
    "err": err_series,
}, separators=(',', ':'))

# ── 生成 HTML ──────────────────────────────────────────────────────────────────
html = f"""<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GNSS SPP Performance Analysis</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
*{{margin:0;padding:0;box-sizing:border-box}}
:root{{--bg:#0a0e17;--surface:#131927;--border:#1e2940;--text:#e0e6f0;--dim:#6b7a99;--accent:#3b82f6;--green:#10b981;--orange:#f59e0b;--red:#ef4444}}
body{{font-family:system-ui,sans-serif;background:var(--bg);color:var(--text);overflow-x:hidden}}
h1{{font-size:1.5rem;font-weight:700;background:linear-gradient(135deg,#60a5fa,#a78bfa);-webkit-background-clip:text;-webkit-text-fill-color:transparent;padding:24px 32px 4px}}
h2{{font-size:.75rem;font-weight:600;color:var(--dim);margin-bottom:12px;text-transform:uppercase;letter-spacing:.08em}}
.subtitle{{color:var(--dim);font-size:.85rem;padding:0 32px 20px}}
.grid{{display:grid;gap:20px;padding:20px 32px}}
.row2{{grid-template-columns:1fr 1fr}}
.card{{background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:20px;overflow:hidden}}
.map-card{{height:420px;padding:0;border-radius:12px}}
.map-card #map{{height:100%;width:100%;border-radius:12px}}
table{{width:100%;border-collapse:collapse;font-size:.85rem}}
th{{text-align:left;color:var(--dim);font-weight:500;padding:8px 12px;border-bottom:1px solid var(--border);font-size:.72rem;text-transform:uppercase;letter-spacing:.05em}}
td{{padding:8px 12px;border-bottom:1px solid var(--border);font-family:monospace;font-size:.82rem}}
.vg{{color:var(--green)}}.vo{{color:var(--orange)}}.vb{{color:var(--red)}}
canvas{{max-height:240px}}
.note{{color:var(--dim);font-size:.75rem;margin-top:10px;font-style:italic}}
@media(max-width:900px){{.row2{{grid-template-columns:1fr}}}}
</style></head><body>
<h1>GNSS SPP Performance Analysis</h1>
<p class="subtitle">自采数据集 &middot; SPP vs u-blox PVT &middot; GPS L1 &middot; 无地面真值</p>

<div class="grid row2">
  <div class="card map-card"><div id="map"></div></div>
  <div class="card">
    <h2>位置统计（SPP vs u-blox PVT）</h2>
    <table>
      <thead><tr><th>指标</th><th>u-blox PVT</th><th>SPP</th></tr></thead>
      <tbody>
        <tr><td>有效点数</td><td class="vg">{len(pvt_pts)}</td><td class="vo">{len(spp_pts)}</td></tr>
        <tr><td>中心纬度</td><td class="vg" colspan="2">{clat:.5f}°N</td></tr>
        <tr><td>中心经度</td><td class="vg" colspan="2">{clon:.5f}°E</td></tr>
        <tr><td>SPP–PVT 均值偏差</td><td class="vg">参考基准</td><td class="vo">{st['mean']:.1f} m</td></tr>
        <tr><td>SPP–PVT 中位偏差</td><td class="vg">参考基准</td><td class="vo">{st['median']:.1f} m</td></tr>
        <tr><td>SPP–PVT RMS</td><td class="vg">参考基准</td><td class="vb">{st['rms']:.1f} m</td></tr>
        <tr><td>SPP–PVT 95th 百分位</td><td class="vg">参考基准</td><td class="vb">{st['p95']:.1f} m</td></tr>
        <tr><td>SPP–PVT 最大偏差</td><td class="vg">参考基准</td><td class="vb">{st['mx']:.1f} m</td></tr>
      </tbody>
    </table>
    <p class="note">⚠️ 此处误差为 SPP 与 u-blox PVT 之差，非与地面真值之差。u-blox PVT 本身有 ~3–10m 误差。</p>
  </div>
</div>

<div class="grid">
  <div class="card">
    <h2>SPP–PVT 逐点偏差时序（米）</h2>
    <canvas id="errChart"></canvas>
  </div>
</div>

<script>
const D={{DATA_PLACEHOLDER}};
const map=L.map('map').setView([{clat},{clon}],16);
L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={{x}}&y={{y}}&z={{z}}',{{maxZoom:21,attribution:'Google'}}).addTo(map);
const pvtL=L.polyline(D.pvt,{{color:'#f59e0b',weight:2.5,opacity:.85}}).addTo(map);
const sppL=L.polyline(D.spp,{{color:'#3b82f6',weight:2,opacity:.8,dashArray:'6,4'}}).addTo(map);
if(D.spp.length){{
  L.circleMarker(D.spp[0],{{radius:6,color:'#1565C0',fillColor:'#4CAF50',fillOpacity:1}}).bindTooltip('起点').addTo(map);
  L.circleMarker(D.spp[D.spp.length-1],{{radius:6,color:'#1565C0',fillColor:'#F44336',fillOpacity:1}}).bindTooltip('终点').addTo(map);
}}
L.control.layers(null,{{'u-blox PVT (橙)':pvtL,'SPP (蓝虚线)':sppL}},{{collapsed:false}}).addTo(map);
map.fitBounds(pvtL.getBounds().pad(.1));

const cDef={{responsive:true,plugins:{{legend:{{labels:{{color:'#6b7a99',font:{{size:11}}}}}}}},scales:{{x:{{ticks:{{color:'#6b7a99',maxTicksLimit:10}},grid:{{color:'#1e2940'}}}},y:{{ticks:{{color:'#6b7a99'}},grid:{{color:'#1e2940'}}}}}}}};
new Chart(document.getElementById('errChart'),{{type:'line',data:{{
  labels:D.err.map(e=>e.t),
  datasets:[{{label:'SPP–PVT 偏差 (m)',data:D.err.map(e=>Math.min(e.d,500)),borderColor:'#3b82f6',borderWidth:1.5,pointRadius:0,fill:{{target:'origin',above:'#3b82f620'}}}}]
}},options:{{...cDef,scales:{{x:{{...cDef.scales.x,title:{{display:true,text:'点序号',color:'#6b7a99'}}}},y:{{...cDef.scales.y,title:{{display:true,text:'偏差 (m，上限500)',color:'#6b7a99'}}}}}}}}}});
</script></body></html>"""

html = html.replace('{DATA_PLACEHOLDER}', DATA)

with open(HTML_PATH, 'w') as f:
    f.write(html)

print(f'完成！')
print(f'  SPP: {len(spp_pts)} 点，PVT: {len(pvt_pts)} 点')
print(f'  SPP–PVT 均值偏差: {st["mean"]:.1f} m，RMS: {st["rms"]:.1f} m')
print(f'  HTML: {HTML_PATH}')
