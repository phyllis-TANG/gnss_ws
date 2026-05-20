#!/usr/bin/env python3
"""
lidar_step5_compare.py
将 LiDAR 射线追踪预测（Step 4）与 del2AINLOS 机器学习标签对比，
计算精确率/召回率/F1，生成 HTML 可视化报告。

输入：
  --lidar  /root/lidar_nlos_prediction.csv  (Step 4 输出)
  --labels /root/data/UrbanNavMedium/nlos_labels_clean.csv  (del2AINLOS 输出)
           列：epoch, sat_id, sys, cn0, elevation, residual, doppler,
               norm_residual, dd_bias, nlos_label

输出：
  --out    /root/lidar_nlos_comparison.html

匹配策略：
  - 用 (sat_id, unix_t) 对齐两个数据集
  - del2AINLOS 的 epoch 列格式可能是 GPS TOW（秒）或 unix 时间，
    脚本会自动检测并对齐（容差 2 秒）
"""

import argparse, csv, math
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--lidar',  default='/root/lidar_nlos_prediction.csv')
ap.add_argument('--labels', default='/root/data/UrbanNavMedium/nlos_labels_clean.csv')
ap.add_argument('--out',    default='/root/lidar_nlos_comparison.html')
ap.add_argument('--tol',    type=float, default=2.0,
                help='时间匹配容差（秒），默认 2s')
args = ap.parse_args()

# ── 读 LiDAR 预测 ─────────────────────────────────────────────────
print('读取 LiDAR 射线追踪预测...')
lidar_rows = []
with open(args.lidar) as f:
    for row in csv.DictReader(f):
        lidar_rows.append({
            'unix_t':   float(row['unix_t']),
            'utc_t':    float(row['utc_t']),
            'sat_id':   row['sat_id'],
            'sys':      row['sys'],
            'elev':     float(row['elevation_deg']),
            'azim':     float(row['azimuth_deg']),
            'lidar_nlos': int(row['lidar_nlos']),
            'hit_dist': float(row['hit_dist_m']),
        })
print(f'  {len(lidar_rows)} 条预测记录')

# ── 读 del2AINLOS 标签 ────────────────────────────────────────────
print('读取 del2AINLOS 标签...')
label_rows = []
with open(args.labels) as f:
    for row in csv.DictReader(f):
        epoch_val = float(row.get('epoch', row.get('unix_t', 0)))
        sat_id = row.get('sat_id', row.get('sat', ''))
        nlos_label = int(float(row.get('nlos_label', row.get('label', 0))))
        cn0 = float(row.get('cn0', 0))
        elev = float(row.get('elevation', row.get('elevation_deg', 0)))
        label_rows.append({
            'epoch': epoch_val,
            'sat_id': sat_id,
            'nlos_label': nlos_label,
            'cn0': cn0,
            'elev': elev,
        })
print(f'  {len(label_rows)} 条标签记录')

# 自动检测 epoch 列是 GPS TOW 还是 unix 时间
# UrbanNav 2021-05-17 02:33 GPS 时间 → unix ≈ 1.62e9；GPS TOW ≈ 95000~96000
if label_rows:
    sample_epoch = label_rows[0]['epoch']
    if sample_epoch < 1e8:
        # 看起来像 GPS TOW，转成 unix time
        # GPS TOW → unix: GPS_EPOCH + week * 604800 + tow
        # 2021-05-17 对应 GPS week 2157 (mod 1024 = 133)
        # 先用 lidar unix_t 推算 week
        lidar_utc0 = lidar_rows[0]['utc_t'] if lidar_rows else 1621218800.0
        GPS_EPOCH_UNIX = 315964800
        total_gps_sec = lidar_utc0 - GPS_EPOCH_UNIX + 18  # 加回闰秒
        gps_week = int(total_gps_sec // 604800)
        print(f'  检测到 GPS TOW 格式，推断 GPS week={gps_week}，转换为 unix 时间')
        for r in label_rows:
            r['epoch'] = GPS_EPOCH_UNIX + gps_week * 604800 + r['epoch'] - 18
    else:
        print(f'  检测到 unix 时间格式（epoch={sample_epoch:.0f}）')

# 构建 (sat_id, epoch) → label 查找结构
# 按 sat_id 分组，epoch 排序，便于近邻匹配
from collections import defaultdict
label_by_sat = defaultdict(list)
for r in label_rows:
    label_by_sat[r['sat_id']].append(r)
for sat_id in label_by_sat:
    label_by_sat[sat_id].sort(key=lambda x: x['epoch'])

def find_label(sat_id, utc_t, tol):
    """在 label_by_sat 中找最近时间的标签，超过容差返回 None。"""
    lst = label_by_sat.get(sat_id, [])
    if not lst:
        return None
    # 二分查找
    lo, hi = 0, len(lst) - 1
    best = None
    best_dt = tol + 1
    while lo <= hi:
        mid = (lo + hi) // 2
        dt = abs(lst[mid]['epoch'] - utc_t)
        if dt < best_dt:
            best_dt = dt
            best = lst[mid]
        if lst[mid]['epoch'] < utc_t:
            lo = mid + 1
        else:
            hi = mid - 1
    return best if best_dt <= tol else None

# ── 匹配并计算混淆矩阵 ───────────────────────────────────────────
print(f'\n匹配预测与标签（容差 {args.tol}s）...')
tp = fp = tn = fn = 0
matched_rows = []
unmatched = 0

for row in lidar_rows:
    lbl = find_label(row['sat_id'], row['utc_t'], args.tol)
    if lbl is None:
        unmatched += 1
        continue

    pred = row['lidar_nlos']
    true = lbl['nlos_label']

    if pred == 1 and true == 1:
        tp += 1
    elif pred == 1 and true == 0:
        fp += 1
    elif pred == 0 and true == 0:
        tn += 1
    else:
        fn += 1

    matched_rows.append({
        'utc_t':       row['utc_t'],
        'sat_id':      row['sat_id'],
        'sys':         row['sys'],
        'elev':        row['elev'],
        'azim':        row['azim'],
        'lidar_nlos':  pred,
        'true_nlos':   true,
        'hit_dist':    row['hit_dist'],
        'cn0':         lbl['cn0'],
    })

total_matched = tp + fp + tn + fn
print(f'  匹配成功: {total_matched}，未匹配（无标签）: {unmatched}')
print(f'\n--- 混淆矩阵 ---')
print(f'  TP={tp}  FP={fp}')
print(f'  FN={fn}  TN={tn}')

precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
f1        = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
accuracy  = (tp + tn) / total_matched if total_matched > 0 else 0.0

print(f'\n--- 指标（LiDAR NLOS 检测）---')
print(f'  精确率 Precision: {precision:.3f}')
print(f'  召回率 Recall:    {recall:.3f}')
print(f'  F1 score:         {f1:.3f}')
print(f'  准确率 Accuracy:  {accuracy:.3f}')

# ── 按仰角分组统计 ────────────────────────────────────────────────
elev_bins = [(0, 15), (15, 30), (30, 45), (45, 90)]
bin_stats = []
for lo, hi in elev_bins:
    sub = [r for r in matched_rows if lo <= r['elev'] < hi]
    if not sub:
        bin_stats.append({'range': f'{lo}–{hi}°', 'n': 0, 'p': 0, 'r': 0, 'f1': 0})
        continue
    stp = sum(1 for r in sub if r['lidar_nlos']==1 and r['true_nlos']==1)
    sfp = sum(1 for r in sub if r['lidar_nlos']==1 and r['true_nlos']==0)
    stn = sum(1 for r in sub if r['lidar_nlos']==0 and r['true_nlos']==0)
    sfn = sum(1 for r in sub if r['lidar_nlos']==0 and r['true_nlos']==1)
    sp = stp / (stp + sfp) if (stp + sfp) > 0 else 0
    sr = stp / (stp + sfn) if (stp + sfn) > 0 else 0
    sf1 = 2*sp*sr/(sp+sr) if (sp+sr) > 0 else 0
    bin_stats.append({'range': f'{lo}–{hi}°', 'n': len(sub), 'p': sp, 'r': sr, 'f1': sf1})

# ── 生成 HTML 报告 ────────────────────────────────────────────────
print(f'\n生成 HTML 报告...')

# 时序数据：按 utc_t 汇总每个历元的 NLOS 比例
from collections import OrderedDict
epoch_data = defaultdict(lambda: {'lidar_nlos': [], 'true_nlos': []})
for r in matched_rows:
    t = round(r['utc_t'])
    epoch_data[t]['lidar_nlos'].append(r['lidar_nlos'])
    epoch_data[t]['true_nlos'].append(r['true_nlos'])

times_sorted = sorted(epoch_data.keys())
lidar_ratio = [np.mean(epoch_data[t]['lidar_nlos']) for t in times_sorted]
true_ratio  = [np.mean(epoch_data[t]['true_nlos'])  for t in times_sorted]
t_rel = [t - times_sorted[0] for t in times_sorted]

# 散点数据：方位角/仰角，颜色=TP/FP/TN/FN
scatter_data = []
for r in matched_rows[:3000]:  # 限制前3000条避免HTML太大
    if   r['lidar_nlos']==1 and r['true_nlos']==1: cat = 'TP'
    elif r['lidar_nlos']==1 and r['true_nlos']==0: cat = 'FP'
    elif r['lidar_nlos']==0 and r['true_nlos']==0: cat = 'TN'
    else:                                            cat = 'FN'
    scatter_data.append({'az': r['azim'], 'el': r['elev'], 'cat': cat,
                         'sat': r['sat_id'], 'dist': r['hit_dist']})

color_map = {'TP': '#2ecc71', 'FP': '#e74c3c', 'TN': '#3498db', 'FN': '#f39c12'}

scatter_js = '[' + ','.join(
    f'{{x:{d["az"]:.1f},y:{d["el"]:.1f},cat:"{d["cat"]}",sat:"{d["sat"]}",dist:{d["dist"]:.1f}}}'
    for d in scatter_data) + ']'

html = f"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="UTF-8">
<title>LiDAR NLOS 对比报告</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
body{{font-family:sans-serif;max-width:1100px;margin:0 auto;padding:20px;background:#f5f7fa}}
h1{{color:#2c3e50}}h2{{color:#34495e;border-bottom:2px solid #3498db;padding-bottom:4px}}
.metrics{{display:flex;gap:16px;flex-wrap:wrap;margin:16px 0}}
.metric{{background:#fff;border-radius:8px;padding:16px 24px;box-shadow:0 2px 6px rgba(0,0,0,.1);text-align:center;min-width:120px}}
.metric .val{{font-size:2em;font-weight:bold;color:#2980b9}}
.metric .lbl{{color:#7f8c8d;font-size:.85em}}
table{{width:100%;border-collapse:collapse;background:#fff;border-radius:8px;overflow:hidden;box-shadow:0 2px 6px rgba(0,0,0,.1)}}
th{{background:#2c3e50;color:#fff;padding:10px 14px;text-align:left}}
td{{padding:9px 14px;border-bottom:1px solid #eee}}
tr:last-child td{{border:none}}
.chart-wrap{{background:#fff;border-radius:8px;padding:20px;box-shadow:0 2px 6px rgba(0,0,0,.1);margin:16px 0}}
</style></head><body>
<h1>LiDAR 射线追踪 NLOS 预测 vs del2AINLOS 标签</h1>
<p>匹配记录：<b>{total_matched}</b> | 未匹配（无标签）：{unmatched}</p>

<h2>整体指标</h2>
<div class="metrics">
  <div class="metric"><div class="val">{precision:.3f}</div><div class="lbl">Precision</div></div>
  <div class="metric"><div class="val">{recall:.3f}</div><div class="lbl">Recall</div></div>
  <div class="metric"><div class="val">{f1:.3f}</div><div class="lbl">F1 Score</div></div>
  <div class="metric"><div class="val">{accuracy:.3f}</div><div class="lbl">Accuracy</div></div>
  <div class="metric"><div class="val">{tp}</div><div class="lbl">TP</div></div>
  <div class="metric"><div class="val">{fp}</div><div class="lbl">FP</div></div>
  <div class="metric"><div class="val">{fn}</div><div class="lbl">FN</div></div>
  <div class="metric"><div class="val">{tn}</div><div class="lbl">TN</div></div>
</div>

<h2>按仰角分层统计</h2>
<table>
<tr><th>仰角范围</th><th>样本数</th><th>Precision</th><th>Recall</th><th>F1</th></tr>
{''.join(f"<tr><td>{b['range']}</td><td>{b['n']}</td><td>{b['p']:.3f}</td><td>{b['r']:.3f}</td><td>{b['f1']:.3f}</td></tr>" for b in bin_stats)}
</table>

<h2>NLOS 比例时序对比</h2>
<div class="chart-wrap"><canvas id="tsChart" height="80"></canvas></div>

<h2>方位角-仰角分布（TP/FP/TN/FN，最多3000条）</h2>
<div class="chart-wrap"><canvas id="scChart" height="90"></canvas></div>

<script>
const tRel = {t_rel};
const lidarRatio = {[f'{v:.4f}' for v in lidar_ratio]};
const trueRatio  = {[f'{v:.4f}' for v in true_ratio]};

new Chart(document.getElementById('tsChart'), {{
  type: 'line',
  data: {{
    labels: tRel,
    datasets: [
      {{label:'LiDAR NLOS 比例', data:lidarRatio, borderColor:'#e74c3c', backgroundColor:'rgba(231,76,60,.1)', tension:.3, pointRadius:0}},
      {{label:'del2AINLOS 标签比例', data:trueRatio, borderColor:'#2980b9', backgroundColor:'rgba(41,128,185,.1)', tension:.3, pointRadius:0}},
    ]
  }},
  options:{{plugins:{{legend:{{position:'top'}}}}, scales:{{x:{{title:{{display:true,text:'相对时间（秒）'}}}}, y:{{title:{{display:true,text:'NLOS 比例'}}, min:0, max:1}}}}}}
}});

const scRaw = {scatter_js};
const colors = {{TP:'#2ecc71',FP:'#e74c3c',TN:'#3498db',FN:'#f39c12'}};
const cats = ['TP','FP','TN','FN'];
const scDatasets = cats.map(cat => ({{
  label: cat,
  data: scRaw.filter(d=>d.cat===cat).map(d=>{{return{{x:d.az,y:d.el,sat:d.sat,dist:d.dist}}}}),
  backgroundColor: colors[cat]+'88',
  borderColor: colors[cat],
  pointRadius: 3,
}}));
new Chart(document.getElementById('scChart'), {{
  type: 'scatter',
  data: {{datasets: scDatasets}},
  options: {{
    plugins: {{legend:{{position:'top'}}, tooltip:{{callbacks:{{label:ctx=>{{const d=ctx.raw;return `${{d.sat}} az=${{ctx.parsed.x.toFixed(0)}}° el=${{ctx.parsed.y.toFixed(0)}}° dist=${{d.dist.toFixed(0)}}m`}}}}}}}},
    scales: {{
      x:{{title:{{display:true,text:'方位角 (°，从北顺时针)'}}, min:0, max:360}},
      y:{{title:{{display:true,text:'仰角 (°)'}}, min:0, max:90}}
    }}
  }}
}});
</script>
</body></html>"""

with open(args.out, 'w', encoding='utf-8') as f:
    f.write(html)

print(f'已保存：{args.out}')
print('\n拷到宿主机：')
print(f'  sudo docker cp ros1_gnss:/root/lidar_nlos_comparison.html ~/lidar_nlos_comparison.html')
