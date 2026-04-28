#!/usr/bin/env python3
"""
generate_analysis.py
读取 /root/trajectory.csv，生成 SPP 分析 HTML

用法：
  python3 /root/generate_analysis.py                           # 仅 SPP 轨迹
  python3 /root/generate_analysis.py --gt ground_truth.txt    # SPP vs 地面真值
  python3 /root/generate_analysis.py --csv /path/to/traj.csv  # 自定义 CSV 路径
  python3 /root/generate_analysis.py --out /path/output.html  # 自定义输出路径

地面真值格式（UrbanNav / TAS Lab 标准）：
  前两行为表头，之后每行：
  UTCTime  Week  GPSTime  Latitude  Longitude  H-Ell  ...
"""

import argparse, csv, json, math, os, sys

ap = argparse.ArgumentParser()
ap.add_argument('--csv', default='/root/trajectory.csv')
ap.add_argument('--gt',  default=None, help='地面真值文件（UrbanNav/TAS Lab 格式）')
ap.add_argument('--out', default='/root/spp_analysis.html')
args = ap.parse_args()

# ── 读 SPP 轨迹 ───────────────────────────────────────────────────────────────
spp_pts = []      # [[lat, lon], ...]
spp_times = []    # [unix_timestamp, ...]

with open(args.csv) as f:
    reader = csv.DictReader(f)
    has_timestamp = 'timestamp' in reader.fieldnames
    for row in reader:
        if row.get('source', '') != 'spp':
            continue
        lat, lon = float(row['lat']), float(row['lon'])
        if abs(lat) < 0.1:
            continue
        spp_pts.append([lat, lon])
        if has_timestamp:
            spp_times.append(float(row['timestamp']))

if not spp_pts:
    print('没有 SPP 数据')
    sys.exit(1)

print(f'SPP 轨迹: {len(spp_pts)} 个点')

# ── 读地面真值 ─────────────────────────────────────────────────────────────────
gt_pts   = []   # [[lat, lon], ...]
gt_times = []   # [unix_timestamp, ...]
has_gt   = False

if args.gt and os.path.exists(args.gt):
    try:
        with open(args.gt) as f:
            lines = f.readlines()
        # 跳过前两行（表头 + 单位行）
        skip = 0
        for i, l in enumerate(lines):
            if l.strip() and not l.strip().startswith('#'):
                # 判断是否是数字开头
                first = l.split()[0]
                try:
                    float(first)
                    skip = i
                    break
                except ValueError:
                    skip = i + 1
        # 自动检测坐标格式：
        # 格式A（十进制度）: UTCTime Week GPSTime Lat Lon H-Ell ...  (parts[3]=lat float)
        # 格式B（度分秒）:   UTCTime Week GPSTime D M S D M S H-Ell (parts[3..8]=DMS)
        def parse_dms(d, m, s):
            return float(d) + float(m)/60.0 + float(s)/3600.0

        dms_format = None  # None=未知, True=DMS, False=十进制
        for line in lines[skip:]:
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                utc_t = float(parts[0])
                # 检测格式：如果 parts[3] 是小整数（度数如22）且 parts[4] 也是小整数（分如18），判定为DMS
                if dms_format is None:
                    p3, p4 = float(parts[3]), float(parts[4])
                    # DMS: parts[3]∈[0,180], parts[4]∈[0,60], parts[5]∈[0,60]
                    # 十进制: parts[3]∈[-90,90] 通常带小数
                    if (p3 == int(p3) and p4 == int(p4) and 0 <= p4 < 60
                            and len(parts) >= 10):
                        dms_format = True
                    else:
                        dms_format = False
                if dms_format:
                    if len(parts) < 10:
                        continue
                    lat = parse_dms(parts[3], parts[4], parts[5])
                    lon = parse_dms(parts[6], parts[7], parts[8])
                else:
                    lat = float(parts[3])
                    lon = float(parts[4])
                gt_times.append(utc_t)
                gt_pts.append([lat, lon])
            except (ValueError, IndexError):
                continue
        if gt_pts:
            has_gt = True
            print(f'地面真值: {len(gt_pts)} 个点')
        else:
            print(f'[警告] GT 文件解析失败或无有效数据: {args.gt}')
    except Exception as e:
        print(f'[警告] 读取 GT 文件出错: {e}')
elif args.gt:
    print(f'[警告] GT 文件不存在: {args.gt}')

# ── 距离计算 ──────────────────────────────────────────────────────────────────
def haversine(p1, p2):
    R = 6371000
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    a = math.sin((lat2-lat1)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
    return 2 * R * math.asin(math.sqrt(a))

# ── SPP vs GT（时间最近邻匹配）或 SPP 相邻散布 ──────────────────────────────────
ref_label = ''
ref_pts   = []
dists     = []

if has_gt and spp_times and gt_times:
    # 逐个 SPP 点，找时间最近的 GT 点（容限 5 秒）
    ref_pts_matched = []
    for i, ts in enumerate(spp_times):
        best_j   = min(range(len(gt_times)), key=lambda j: abs(gt_times[j] - ts))
        dt       = abs(gt_times[best_j] - ts)
        if dt > 5.0:
            continue
        dists.append(haversine(spp_pts[i], gt_pts[best_j]))
        ref_pts_matched.append(gt_pts[best_j])
    ref_pts   = ref_pts_matched
    ref_label = '地面真值（RTK/INS）'
    print(f'SPP↔GT 时间匹配: {len(dists)}/{len(spp_pts)} 点（容限 5s）')
elif has_gt and not spp_times:
    # 无时间戳时按索引配对（1 Hz SPP vs 10 Hz GT → 每10个GT取1个）
    step = max(1, len(gt_pts) // len(spp_pts)) if spp_pts else 1
    gt_sub = gt_pts[::step][:len(spp_pts)]
    n_pair = min(len(spp_pts), len(gt_sub))
    dists     = [haversine(spp_pts[i], gt_sub[i]) for i in range(n_pair)]
    ref_pts   = gt_sub
    ref_label = '地面真值（索引配对）'

if not dists:
    # 仅 SPP 散布（相邻点间距）
    dists = [haversine(spp_pts[i], spp_pts[i+1]) for i in range(len(spp_pts)-1)]
    ref_label = ''

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

st   = stats(dists)
clat = sum(p[0] for p in spp_pts) / len(spp_pts)
clon = sum(p[1] for p in spp_pts) / len(spp_pts)

step = max(1, len(dists) // 200)
err_series = [{"t": i, "d": round(dists[i], 2)} for i in range(0, len(dists), step)]

# ── 判断标注文字 ───────────────────────────────────────────────────────────────
if has_gt:
    subtitle_tag = 'SPP vs 地面真值（RTK/INS）'
    note_text    = '误差为 SPP 与 RTK/INS 地面真值之差。'
    ref_col_hdr  = '地面真值'
    metric_label = 'SPP–GT'
else:
    subtitle_tag = 'SPP 轨迹（无地面真值）'
    note_text    = '⚠️ 无地面真值文件。指标为相邻 SPP 点间距（散布），非定位误差。<br>如有 UrbanNav GT 文件，运行: python3 generate_analysis.py --gt <gt_file.txt>'
    ref_col_hdr  = '—'
    metric_label = 'SPP 散布'

# ── 嵌入数据 ───────────────────────────────────────────────────────────────────
DATA = json.dumps({
    "spp": spp_pts,
    "ref": ref_pts,
    "err": err_series,
}, separators=(',', ':'))

# ── HTML ───────────────────────────────────────────────────────────────────────
n_ref_str = str(len(ref_pts)) if ref_pts else '—'

html = f"""<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GNSS SPP Performance Analysis</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css"/>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js"></script>
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
<p class="subtitle">UrbanNav Medium-Urban-1 &middot; {subtitle_tag} &middot; GPS L1</p>

<div class="grid row2">
  <div class="card map-card"><div id="map"></div></div>
  <div class="card">
    <h2>位置统计（{metric_label}）</h2>
    <table>
      <thead><tr><th>指标</th><th>{ref_col_hdr}</th><th>SPP</th></tr></thead>
      <tbody>
        <tr><td>有效点数</td><td class="vg">{n_ref_str}</td><td class="vo">{len(spp_pts)}</td></tr>
        <tr><td>中心纬度</td><td class="vg" colspan="2">{clat:.6f}°N</td></tr>
        <tr><td>中心经度</td><td class="vg" colspan="2">{clon:.6f}°E</td></tr>
        <tr><td>{metric_label} 均值</td><td class="vg">{'参考基准' if has_gt else '—'}</td><td class="vo">{st['mean']:.1f} m</td></tr>
        <tr><td>{metric_label} 中位值</td><td class="vg">{'参考基准' if has_gt else '—'}</td><td class="vo">{st['median']:.1f} m</td></tr>
        <tr><td>{metric_label} RMS</td><td class="vg">{'参考基准' if has_gt else '—'}</td><td class="vb">{st['rms']:.1f} m</td></tr>
        <tr><td>{metric_label} 95th 百分位</td><td class="vg">{'参考基准' if has_gt else '—'}</td><td class="vb">{st['p95']:.1f} m</td></tr>
        <tr><td>{metric_label} 最大值</td><td class="vg">{'参考基准' if has_gt else '—'}</td><td class="vb">{st['mx']:.1f} m</td></tr>
      </tbody>
    </table>
    <p class="note">{note_text}</p>
  </div>
</div>

<div class="grid">
  <div class="card">
    <h2>{metric_label} 逐点偏差时序（米）</h2>
    <canvas id="errChart"></canvas>
  </div>
</div>

<script>
const D={{DATA_PLACEHOLDER}};
const map=L.map('map').setView([{clat},{clon}],16);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{maxZoom:19,attribution:'© OpenStreetMap'}}).addTo(map);
const sppL=L.polyline(D.spp,{{color:'#3b82f6',weight:2.5,opacity:.85,dashArray:'6,4'}}).addTo(map);
const layers={{'SPP (蓝虚线)':sppL}};
if(D.ref&&D.ref.length>0){{
  const refL=L.polyline(D.ref,{{color:'#10b981',weight:2,opacity:.8}}).addTo(map);
  layers['{ref_col_hdr} (绿)'] = refL;
  map.fitBounds(refL.getBounds().pad(.1));
}}else{{
  map.fitBounds(sppL.getBounds().pad(.1));
}}
if(D.spp.length){{
  L.circleMarker(D.spp[0],{{radius:6,color:'#1565C0',fillColor:'#4CAF50',fillOpacity:1}}).bindTooltip('SPP起点').addTo(map);
  L.circleMarker(D.spp[D.spp.length-1],{{radius:6,color:'#1565C0',fillColor:'#F44336',fillOpacity:1}}).bindTooltip('SPP终点').addTo(map);
}}
L.control.layers(null,layers,{{collapsed:false}}).addTo(map);

const cDef={{responsive:true,plugins:{{legend:{{labels:{{color:'#6b7a99',font:{{size:11}}}}}}}},scales:{{x:{{ticks:{{color:'#6b7a99',maxTicksLimit:10}},grid:{{color:'#1e2940'}}}},y:{{ticks:{{color:'#6b7a99'}},grid:{{color:'#1e2940'}}}}}}}};
new Chart(document.getElementById('errChart'),{{type:'line',data:{{
  labels:D.err.map(e=>e.t),
  datasets:[{{label:'{metric_label} (m)',data:D.err.map(e=>Math.min(e.d,500)),borderColor:'#3b82f6',borderWidth:1.5,pointRadius:0,fill:{{target:'origin',above:'#3b82f620'}}}}]
}},options:{{...cDef,scales:{{x:{{...cDef.scales.x,title:{{display:true,text:'点序号',color:'#6b7a99'}}}},y:{{...cDef.scales.y,title:{{display:true,text:'偏差 (m，上限500)',color:'#6b7a99'}}}}}}}}}}}});
</script></body></html>"""

html = html.replace('{DATA_PLACEHOLDER}', DATA)

with open(args.out, 'w') as f:
    f.write(html)

print(f'完成！')
print(f'  SPP: {len(spp_pts)} 点，参考: {n_ref_str} 点')
if has_gt:
    print(f'  SPP–GT 均值: {st["mean"]:.1f} m，RMS: {st["rms"]:.1f} m，95th: {st["p95"]:.1f} m')
else:
    print(f'  （无地面真值，显示 SPP 散布统计）')
print(f'  HTML: {args.out}')
