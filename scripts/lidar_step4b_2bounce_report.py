#!/usr/bin/env python3
"""
lidar_step4b_2bounce_report.py
对 Step4 的 2-bounce 射线追踪结果做可视化分析，输出 HTML 报告。

输入: lidar_nlos_prediction_2b.csv（Step4 的 2-bounce 输出）
输出: /root/2bounce_report.html

用法:
  python3 lidar_step4b_2bounce_report.py \
    --csv /root/lidar_nlos_prediction_2b.csv \
    --out /root/2bounce_report.html
"""

import argparse, base64, csv, io, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from collections import Counter

ap = argparse.ArgumentParser()
ap.add_argument('--csv', default='/root/lidar_nlos_prediction_2b.csv')
ap.add_argument('--out', default='/root/2bounce_report.html')
args = ap.parse_args()

# ── 读数据 ──────────────────────────────────────────────────────────
rows = []
with open(args.csv) as f:
    for r in csv.DictReader(f):
        rows.append({
            'sys':      r['sys'],
            'elev':     float(r['elevation_deg']),
            'azim':     float(r['azimuth_deg']),
            'nlos':     int(r['lidar_nlos']),
            'n_b':      int(r['n_bounces']),
            'dist1':    float(r['hit_dist_m']),
            'dist2':    float(r['hit_dist2_m']),
            'ne':       float(r['normal_e']),
            'nn':       float(r['normal_n']),
            'nu':       float(r['normal_u']),
        })

total   = len(rows)
los     = [r for r in rows if r['n_b'] == 0]
b1      = [r for r in rows if r['n_b'] == 1]
b2      = [r for r in rows if r['n_b'] == 2]
nlos_all = b1 + b2

print(f'总计: {total}  LOS: {len(los)}  1-bounce: {len(b1)}  2-bounce: {len(b2)}')

# ── 计算 1-bounce ΔL（用法向量） ────────────────────────────────────
def sat_dir(azim_deg, elev_deg):
    az, el = math.radians(azim_deg), math.radians(elev_deg)
    return np.array([math.sin(az)*math.cos(el),
                     math.cos(az)*math.cos(el),
                     math.sin(el)])

delta_L_1b = []
delta_L_2b = []

for r in nlos_all:
    d = sat_dir(r['azim'], r['elev'])
    n = np.array([r['ne'], r['nn'], r['nu']])
    n_norm = np.linalg.norm(n)
    if n_norm > 0.1:
        n = n / n_norm
        cos_theta = abs(float(np.dot(d, n)))
        dl1 = 2.0 * r['dist1'] * cos_theta
    else:
        dl1 = float('nan')
    delta_L_1b.append(dl1)
    # 2-bounce: 总额外路径 ≈ d1 + d2
    delta_L_2b.append(r['dist1'] + r['dist2'])

delta_L_1b = np.array(delta_L_1b)
delta_L_2b = np.array(delta_L_2b)

# ── 绘图工具 ────────────────────────────────────────────────────────
def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110, bbox_inches='tight')
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

imgs = {}

# ── 图1：LOS / 1-bounce / 2-bounce 总览饼图 + 柱状图 ──────────────
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
fig.suptitle('Step4 2-Bounce Ray Casting — 总体分布', fontsize=13)

labels = ['LOS', '1-bounce NLOS', '2-bounce NLOS']
counts = [len(los), len(b1), len(b2)]
colors = ['#4CAF50', '#FF9800', '#F44336']
axes[0].pie(counts, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
axes[0].set_title(f'总计 {total} 条记录')

sys_order = sorted(set(r['sys'] for r in rows))
x = np.arange(len(sys_order))
w = 0.25
for ax_i, (grp, color, label) in enumerate([(los,'#4CAF50','LOS'),
                                              (b1,'#FF9800','1-bounce'),
                                              (b2,'#F44336','2-bounce')]):
    cnt = [sum(1 for r in grp if r['sys']==s) for s in sys_order]
    axes[1].bar(x + ax_i*w, cnt, w, label=label, color=color)
axes[1].set_xticks(x + w)
axes[1].set_xticklabels(sys_order)
axes[1].set_ylabel('卫星-历元数')
axes[1].set_title('按星座分类')
axes[1].legend()
plt.tight_layout()
imgs['overview'] = fig_to_b64(fig)

# ── 图2：仰角分层 × bounce 类型 ────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
fig.suptitle('仰角 vs 反弹类型', fontsize=13)

bins_elev = [0, 15, 30, 45, 60, 90]
labels_e = ['0-15°', '15-30°', '30-45°', '45-60°', '60-90°']

def elev_hist(grp, bins):
    return [sum(1 for r in grp if bins[i] <= r['elev'] < bins[i+1])
            for i in range(len(bins)-1)]

e_los = elev_hist(los, bins_elev)
e_b1  = elev_hist(b1,  bins_elev)
e_b2  = elev_hist(b2,  bins_elev)
x = np.arange(len(labels_e))
w = 0.28
axes[0].bar(x-w, e_los, w, label='LOS',        color='#4CAF50')
axes[0].bar(x,   e_b1,  w, label='1-bounce',   color='#FF9800')
axes[0].bar(x+w, e_b2,  w, label='2-bounce',   color='#F44336')
axes[0].set_xticks(x); axes[0].set_xticklabels(labels_e)
axes[0].set_ylabel('计数'); axes[0].set_title('仰角区间 vs 弹射类型')
axes[0].legend()

# 2-bounce 在 NLOS 中的比例 vs 仰角
e_nlos_total = [e_b1[i]+e_b2[i] for i in range(len(labels_e))]
ratio_2b = [e_b2[i]/max(e_nlos_total[i],1)*100 for i in range(len(labels_e))]
axes[1].bar(labels_e, ratio_2b, color='#F44336', alpha=0.8)
axes[1].set_ylabel('2-bounce 占 NLOS %'); axes[1].set_title('各仰角区间 2-bounce 比例')
axes[1].set_ylim(0, 105)
for i, v in enumerate(ratio_2b):
    axes[1].text(i, v+1.5, f'{v:.0f}%', ha='center', fontsize=9)
plt.tight_layout()
imgs['elev'] = fig_to_b64(fig)

# ── 图3：hit_dist 分布（d1 和 d2）────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
fig.suptitle('反射路径距离分布', fontsize=13)

d1_all = [r['dist1'] for r in nlos_all]
d2_all = [r['dist2'] for r in b2]

axes[0].hist(d1_all, bins=40, color='#FF9800', alpha=0.8, edgecolor='white')
axes[0].set_xlabel('第一击中距离 d1 (m)'); axes[0].set_ylabel('计数')
axes[0].set_title(f'd1 分布  n={len(d1_all)}  mean={np.mean(d1_all):.1f}m')
axes[0].axvline(np.mean(d1_all), color='red', ls='--', label=f'mean={np.mean(d1_all):.1f}m')
axes[0].legend()

axes[1].hist(d2_all, bins=40, color='#F44336', alpha=0.8, edgecolor='white')
axes[1].set_xlabel('第二击中距离 d2 (m)'); axes[1].set_ylabel('计数')
axes[1].set_title(f'd2 分布（2-bounce only）  n={len(d2_all)}  mean={np.mean(d2_all):.1f}m')
axes[1].axvline(np.mean(d2_all), color='darkred', ls='--', label=f'mean={np.mean(d2_all):.1f}m')
axes[1].legend()
plt.tight_layout()
imgs['dist'] = fig_to_b64(fig)

# ── 图4：ΔL 预测分布比较（1-bounce vs 2-bounce）────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
fig.suptitle('额外路径长度 ΔL 预测分布', fontsize=13)

valid_1b = delta_L_1b[~np.isnan(delta_L_1b)]
valid_2b = delta_L_2b

all_bins = np.linspace(0, max(valid_1b.max(), valid_2b.max(), 50), 50)

axes[0].hist(valid_1b, bins=all_bins, color='#FF9800', alpha=0.8, label='1-bounce ΔL=2·d1·cos(θ)')
axes[0].hist(valid_2b, bins=all_bins, color='#F44336', alpha=0.6, label='2-bounce ΔL=d1+d2')
axes[0].set_xlabel('ΔL 预测值 (m)'); axes[0].set_ylabel('计数')
axes[0].set_title('ΔL 分布叠加对比')
axes[0].legend()

# CDF
axes[1].plot(np.sort(valid_1b), np.linspace(0,1,len(valid_1b)),
             color='#FF9800', label=f'1-bounce（mean={valid_1b.mean():.1f}m）')
axes[1].plot(np.sort(valid_2b), np.linspace(0,1,len(valid_2b)),
             color='#F44336', label=f'2-bounce（mean={valid_2b.mean():.1f}m）')
axes[1].set_xlabel('ΔL (m)'); axes[1].set_ylabel('CDF')
axes[1].set_title('ΔL 累积分布（CDF）')
axes[1].legend(); axes[1].grid(True, alpha=0.3)
plt.tight_layout()
imgs['deltaL'] = fig_to_b64(fig)

# ── 图5：表面法向量方向玫瑰图（水平方向 + 竖直分量）────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
fig.suptitle('第一反射面法向量方向分布', fontsize=13)

normals = np.array([[r['ne'], r['nn'], r['nu']] for r in nlos_all
                     if abs(r['ne'])+abs(r['nn'])+abs(r['nu']) > 0.1])
if len(normals) > 0:
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.where(norms>0, norms, 1)
    azim_n = np.degrees(np.arctan2(normals[:,0], normals[:,1])) % 360
    elev_n = np.degrees(np.arcsin(np.clip(normals[:,2], -1, 1)))

    axes[0].hist(azim_n, bins=36, color='steelblue', alpha=0.8)
    axes[0].set_xlabel('法向量方位角 (°, 从正北顺时针)')
    axes[0].set_ylabel('计数'); axes[0].set_title('法向量水平方向分布')

    axes[1].hist(elev_n, bins=30, color='steelblue', alpha=0.8)
    axes[1].set_xlabel('法向量仰角 (°)')
    axes[1].set_ylabel('计数'); axes[1].set_title('法向量竖直分量分布\n（接近0°=垂直墙面，接近90°=水平面）')
    frac_wall = np.mean(np.abs(elev_n) < 30) * 100
    axes[1].axvline(-30, color='red', ls='--', alpha=0.5)
    axes[1].axvline( 30, color='red', ls='--', alpha=0.5, label=f'垂直墙面占比 {frac_wall:.0f}%')
    axes[1].legend()
plt.tight_layout()
imgs['normal'] = fig_to_b64(fig)

# ── 图6：d1 vs d2 散点 + 颜色=仰角 ────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 5.5))
sc = ax.scatter([r['dist1'] for r in b2], [r['dist2'] for r in b2],
                c=[r['elev'] for r in b2], cmap='RdYlGn', s=4, alpha=0.5)
plt.colorbar(sc, ax=ax, label='仰角 (°)')
ax.set_xlabel('d1：第一击中距离 (m)')
ax.set_ylabel('d2：第二击中距离 (m)')
ax.set_title(f'2-bounce: d1 vs d2（n={len(b2)}，按仰角着色）')
corr = np.corrcoef([r['dist1'] for r in b2], [r['dist2'] for r in b2])[0,1]
ax.text(0.05, 0.95, f'r(d1,d2)={corr:.3f}', transform=ax.transAxes,
        va='top', fontsize=10, bbox=dict(boxstyle='round', fc='white', alpha=0.8))
plt.tight_layout()
imgs['scatter'] = fig_to_b64(fig)

# ── 统计摘要 ────────────────────────────────────────────────────────
d1_arr = np.array(d1_all)
d2_arr = np.array(d2_all)
dl_arr = delta_L_2b

stats_html = f"""
<table>
<tr><th></th><th>LOS</th><th>1-bounce NLOS</th><th>2-bounce NLOS</th></tr>
<tr><td>数量</td><td>{len(los)}</td><td>{len(b1)}</td><td>{len(b2)}</td></tr>
<tr><td>占比</td><td>{100*len(los)/total:.1f}%</td>
    <td>{100*len(b1)/total:.1f}%</td><td>{100*len(b2)/total:.1f}%</td></tr>
<tr><td>平均仰角</td>
    <td>{np.mean([r['elev'] for r in los]):.1f}°</td>
    <td>{np.mean([r['elev'] for r in b1]):.1f}°</td>
    <td>{np.mean([r['elev'] for r in b2]):.1f}°</td></tr>
</table>
<table>
<tr><th>路径长度统计</th><th>mean</th><th>std</th><th>50th</th><th>95th</th></tr>
<tr><td>d1（所有NLOS）</td><td>{d1_arr.mean():.1f}m</td><td>{d1_arr.std():.1f}m</td>
    <td>{np.percentile(d1_arr,50):.1f}m</td><td>{np.percentile(d1_arr,95):.1f}m</td></tr>
<tr><td>d2（2-bounce only）</td><td>{d2_arr.mean():.1f}m</td><td>{d2_arr.std():.1f}m</td>
    <td>{np.percentile(d2_arr,50):.1f}m</td><td>{np.percentile(d2_arr,95):.1f}m</td></tr>
<tr><td>ΔL_2b = d1+d2</td><td>{dl_arr.mean():.1f}m</td><td>{dl_arr.std():.1f}m</td>
    <td>{np.percentile(dl_arr,50):.1f}m</td><td>{np.percentile(dl_arr,95):.1f}m</td></tr>
<tr><td>ΔL_1b = 2·d1·cos(θ)</td>
    <td>{np.nanmean(delta_L_1b):.1f}m</td><td>{np.nanstd(delta_L_1b):.1f}m</td>
    <td>{np.nanpercentile(delta_L_1b,50):.1f}m</td><td>{np.nanpercentile(delta_L_1b,95):.1f}m</td></tr>
</table>
"""

# ── 生成 HTML ──────────────────────────────────────────────────────
html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>2-Bounce Ray Casting Report</title>
<style>
body{{font-family:sans-serif;margin:20px;background:#f5f5f5}}
h1{{color:#333}} h2{{color:#555;margin-top:30px}}
img{{max-width:100%;border:1px solid #ddd;border-radius:6px;margin:8px 0}}
table{{border-collapse:collapse;margin:10px 0}}
td,th{{border:1px solid #ccc;padding:6px 12px;text-align:right}}
th{{background:#4a90d9;color:white}}
tr:nth-child(even){{background:#eef}}
.key{{background:#fff3cd;padding:12px;border-left:4px solid #ffc107;margin:10px 0}}
</style></head><body>
<h1>Step4 2-Bounce 射线追踪分析报告</h1>
<div class="key">
<b>关键结论：</b><br>
• 总记录 {total} 条：LOS {len(los)}（{100*len(los)/total:.1f}%）｜
  1-bounce {len(b1)}（{100*len(b1)/total:.1f}%）｜
  2-bounce {len(b2)}（{100*len(b2)/total:.1f}%，占NLOS的{100*len(b2)/max(len(nlos_all),1):.1f}%）<br>
• 2-bounce 平均额外路径 ΔL = d1+d2 = {dl_arr.mean():.1f}m（95th: {np.percentile(dl_arr,95):.1f}m）<br>
• 1-bounce ΔL = 2·d1·cos(θ) 均值 = {np.nanmean(delta_L_1b):.1f}m，2-bounce 估计均值更大
</div>

{stats_html}

<h2>1. 总体分布</h2>
<img src="data:image/png;base64,{imgs['overview']}">

<h2>2. 仰角 × 弹射类型</h2>
<img src="data:image/png;base64,{imgs['elev']}">

<h2>3. 击中距离分布（d1, d2）</h2>
<img src="data:image/png;base64,{imgs['dist']}">

<h2>4. 额外路径 ΔL 预测分布</h2>
<img src="data:image/png;base64,{imgs['deltaL']}">

<h2>5. 第一反射面法向量方向</h2>
<img src="data:image/png;base64,{imgs['normal']}">

<h2>6. 2-bounce: d1 vs d2 散点图</h2>
<img src="data:image/png;base64,{imgs['scatter']}">

</body></html>"""

with open(args.out, 'w') as f:
    f.write(html)
print(f'已保存: {args.out}')
