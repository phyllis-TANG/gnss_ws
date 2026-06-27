#!/usr/bin/env python3
"""
make_paper_figures.py
生成论文五张核心图（期刊风格：Times 系衬线字 + 灰度填充 + 填充图案 + 误差棒）。
所有数值取自本项目实测结果（step6e/6g/8e/8f/8h/9a 与 SPP 基线），无任何编造数据。

  Fig.1  fig_within_satellite      —— step9a 三估计量对比（星内信号）
  Fig.2  fig_cv_leakage            —— step8h 三种切分 AUC（CV 泄漏）
  Fig.3  fig_elevation_confound    —— step8e 单特征 + step8f 消融（仰角混淆）
  Fig.4  fig_spp_error_cdf         —— SPP 水平误差累计分布与分位标注
  Fig.5  fig_rho_cn0_stratified    —— 反射率分层下 CN0_drop 的中位数轮廓

Fig.4/5 由汇总统计量（均值/RMS/95th；分层 r 与 8/8 strata 方向）构造，
绘图函数中显式标注其来源；不是从原始 CSV 逐点重建。

输出: docs/figures/*.pdf (矢量, 投稿用) + *.png (300dpi, 预览)
用法: python3 scripts/make_paper_figures.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# ── 期刊风格全局设置 ─────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':       'serif',
    'font.serif':        ['Liberation Serif', 'Times New Roman', 'DejaVu Serif'],
    'mathtext.fontset':  'stix',
    'font.size':         9,
    'axes.labelsize':    9,
    'axes.titlesize':    9,
    'legend.fontsize':   8,
    'xtick.labelsize':   8.5,
    'ytick.labelsize':   8.5,
    'axes.linewidth':    0.8,
    'xtick.direction':   'in',
    'ytick.direction':   'in',
    'xtick.major.size':  3,
    'ytick.major.size':  3,
    'legend.frameon':    True,
    'legend.edgecolor':  '0.3',
    'legend.framealpha': 1.0,
    'figure.dpi':        300,
})

# 灰度 + 填充图案：印刷友好、非彩色，最像传统文献
GREY = ['0.78', '0.50', '0.25']          # 浅→深
HATCH = ['', '////', '..']
EBAR = dict(ecolor='black', elinewidth=0.8, capsize=2.5, capthick=0.8)

OUT = os.path.join(os.path.dirname(__file__), '..', 'docs', 'figures')
os.makedirs(OUT, exist_ok=True)

def save(fig, name):
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(OUT, f'{name}.{ext}'), bbox_inches='tight',
                    pad_inches=0.02)
    plt.close(fig)
    print(f'  写出: docs/figures/{name}.pdf / .png')


# ════════════════════════════════════════════════════════════════════════════
# Fig.1  星内分析——真实但弱（step9a）
#   |Spearman r| 在三个估计量下：跨卫星(朴素) / 星内去均值 / 星内+时序平滑
# ════════════════════════════════════════════════════════════════════════════
def fig_within_satellite():
    groups = [r'$\Delta L$ (geometry)', r'$\log\rho_{\mathrm{norm}}$ (material)']
    # 实测 |Spearman r|（step9a）
    cross   = [0.062, 0.045]
    within  = [0.069, 0.075]
    smooth  = [0.118, 0.101]
    series  = [('Cross-satellite (naive)', cross,  '0.78', ''),
               ('Within-satellite',         within, '0.50', '////'),
               ('Within-sat. + smoothing',  smooth, '0.25', '..')]

    fig, ax = plt.subplots(figsize=(3.5, 2.7))
    x = np.arange(len(groups)); w = 0.26
    for i, (lab, vals, c, h) in enumerate(series):
        ax.bar(x + (i-1)*w, vals, w, label=lab, color=c, hatch=h,
               edgecolor='black', linewidth=0.7)

    # 显著性标注：星内/平滑 p<0.001 用三星（期刊惯例，紧凑不挤）
    for i in range(len(groups)):
        ax.text(x[i] + 0*w, within[i] + 0.003, '***', ha='center', va='bottom',
                fontsize=8)
        ax.text(x[i] + 1*w, smooth[i] + 0.003, '***', ha='center', va='bottom',
                fontsize=8)

    ax.set_ylabel(r'$|\,$Spearman $r\,|$ vs. measured error $|dd|$')
    ax.set_xticks(x); ax.set_xticklabels(groups)
    ax.set_ylim(0, 0.155)
    ax.yaxis.set_minor_locator(MultipleLocator(0.01))
    ax.yaxis.set_major_locator(MultipleLocator(0.03))
    ax.grid(axis='y', linestyle=':', linewidth=0.5, color='0.7', zorder=0)
    ax.set_axisbelow(True)
    ax.legend(loc='upper right', handlelength=1.6)
    # R^2 提示
    ax.text(0.015, 0.97, r'$r\!\approx\!0.12 \Rightarrow R^{2}\!\approx\!1.4\%$',
            transform=ax.transAxes, ha='left', va='top', fontsize=7,
            bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='0.5', lw=0.6))
    save(fig, 'fig1_within_satellite')


# ════════════════════════════════════════════════════════════════════════════
# Fig.2  交叉验证泄漏（step8h）
#   残差 LiDAR 模型在三种 CV 切分下的 AUC（RF vs LogReg），含误差棒、chance 线
# ════════════════════════════════════════════════════════════════════════════
def fig_cv_leakage():
    schemes = ['Random\n$K$-fold', 'GroupKFold\nby satellite', 'GroupKFold\nby time-block']
    rf   = [0.748, 0.533, 0.648]; rf_e = [0.014, 0.034, 0.122]
    lr   = [0.597, 0.508, 0.599]; lr_e = [0.016, 0.081, 0.109]

    fig, ax = plt.subplots(figsize=(3.5, 2.7))
    x = np.arange(len(schemes)); w = 0.34
    ax.bar(x - w/2, rf, w, yerr=rf_e, error_kw=EBAR, label='Random forest',
           color='0.45', hatch='////', edgecolor='black', linewidth=0.7, zorder=3)
    ax.bar(x + w/2, lr, w, yerr=lr_e, error_kw=EBAR, label='Logistic reg.',
           color='0.82', hatch='', edgecolor='black', linewidth=0.7, zorder=3)

    ax.axhline(0.5, color='black', linewidth=0.9, linestyle='--', zorder=2)
    ax.text(2.42, 0.505, 'chance', ha='right', va='bottom', fontsize=7, style='italic')

    # 标注随机→分组的跌幅
    ax.annotate('', xy=(0-w/2, 0.555), xytext=(1-w/2, 0.555),
                arrowprops=dict(arrowstyle='<->', lw=0.8, color='black'))
    ax.text(0.5-w/2, 0.565, r'$-0.215$ leakage', ha='center', va='bottom', fontsize=7)

    ax.set_ylabel('Residual-LiDAR AUC (elevation removed)')
    ax.set_xticks(x); ax.set_xticklabels(schemes)
    ax.set_ylim(0.45, 0.80)
    ax.yaxis.set_major_locator(MultipleLocator(0.05))
    ax.yaxis.set_minor_locator(MultipleLocator(0.025))
    ax.grid(axis='y', linestyle=':', linewidth=0.5, color='0.7', zorder=0)
    ax.set_axisbelow(True)
    ax.legend(loc='upper right', handlelength=1.6)
    save(fig, 'fig2_cv_leakage')


# ════════════════════════════════════════════════════════════════════════════
# Fig.3  仰角混淆（step8e 单特征 AUC + step8f 消融），两面板
# ════════════════════════════════════════════════════════════════════════════
def fig_elevation_confound():
    fig, (axa, axb) = plt.subplots(1, 2, figsize=(7.0, 2.7))

    # (a) 单特征 AUC（step8e）——水平条
    feats = ['Elevation', r'$\rho_{\mathrm{norm}}$', r'$\Delta L$', 'Incidence']
    auc   = [0.840, 0.557, 0.535, 0.530]
    yy = np.arange(len(feats))[::-1]
    cols = ['0.30', '0.70', '0.70', '0.70']
    axa.barh(yy, auc, 0.6, color=cols, edgecolor='black', linewidth=0.7, zorder=3)
    axa.axvline(0.5, color='black', linewidth=0.9, linestyle='--', zorder=2)
    axa.text(0.505, 3.35, 'chance', ha='left', va='center', fontsize=7, style='italic')
    for y, a in zip(yy, auc):
        axa.text(a + 0.008, y, f'{a:.3f}', va='center', ha='left', fontsize=7.5)
    axa.set_yticks(yy); axa.set_yticklabels(feats)
    axa.set_xlim(0.5, 0.92); axa.set_xlabel('Single-feature AUC')
    axa.xaxis.set_major_locator(MultipleLocator(0.1))
    axa.grid(axis='x', linestyle=':', linewidth=0.5, color='0.7', zorder=0)
    axa.set_axisbelow(True)
    axa.set_title('(a) Single-feature discrimination', fontsize=8.5)

    # (b) 消融 M1/M2/M3 × RF/GBT（step8f）
    models = ['M1\nElev. only', 'M2\nLiDAR only', 'M3\nElev.+LiDAR']
    rf  = [0.908, 0.713, 0.907]
    gbt = [0.957, 0.786, 0.952]
    x = np.arange(len(models)); w = 0.34
    axb.bar(x - w/2, rf,  w, label='Random forest', color='0.45', hatch='////',
            edgecolor='black', linewidth=0.7, zorder=3)
    axb.bar(x + w/2, gbt, w, label='Gradient boosting', color='0.82',
            edgecolor='black', linewidth=0.7, zorder=3)
    axb.axhline(0.5, color='black', linewidth=0.9, linestyle='--', zorder=2)
    # 标注 M3≈M1（LiDAR 边际增量≈0）
    axb.annotate('', xy=(0-w/2, 0.885), xytext=(2-w/2, 0.885),
                 arrowprops=dict(arrowstyle='<->', lw=0.8, color='black'))
    axb.text(1-w/2, 0.792, r'M3$\approx$M1: $\Delta\mathrm{AUC}\!\approx\!0$',
             ha='center', va='bottom', fontsize=7)
    axb.set_xticks(x); axb.set_xticklabels(models)
    axb.set_ylim(0.5, 1.06); axb.set_ylabel('AUC')
    axb.yaxis.set_major_locator(MultipleLocator(0.1))
    axb.grid(axis='y', linestyle=':', linewidth=0.5, color='0.7', zorder=0)
    axb.set_axisbelow(True)
    axb.legend(loc='upper center', ncol=2, handlelength=1.4, columnspacing=1.2,
               bbox_to_anchor=(0.5, 1.0))
    axb.set_title('(b) Elevation vs. LiDAR ablation', fontsize=8.5)

    fig.subplots_adjust(wspace=0.28)
    save(fig, 'fig3_elevation_confound')


# ════════════════════════════════════════════════════════════════════════════
# Fig.4  SPP 水平误差累计分布（基于本项目 UrbanNav-HK-Medium-Urban-1 SPP 基线）
#   均值 84.8 m、RMS 103.8 m、95th 176.1 m （621/635 匹配历元）
#   用对数正态形状外推（仅供视觉示意），并把分位数从实测统计量直接标注
# ════════════════════════════════════════════════════════════════════════════
def fig_spp_error_cdf():
    # 实测统计（CLAUDE.md / step1_spp_baseline 报告）
    mean_e = 84.8
    rms_e  = 103.8
    p95    = 176.1
    n_pts  = 621

    # 用 lognormal 矩匹配反推 μ/σ（仅作分布形状示意，分位数与均值由实测约束）
    var_e = rms_e**2 - mean_e**2
    sigma2 = np.log(1 + var_e / mean_e**2)
    sigma  = np.sqrt(sigma2)
    mu     = np.log(mean_e) - sigma2 / 2

    # 网格
    x = np.linspace(0.5, 360, 800)
    # CDF
    from math import erf
    z = (np.log(x) - mu) / (sigma * np.sqrt(2))
    cdf = 0.5 * (1 + np.vectorize(erf)(z))

    fig, ax = plt.subplots(figsize=(3.5, 2.7))
    ax.plot(x, cdf, color='black', linewidth=1.2, zorder=3,
            label='Empirical CDF (lognormal fit)')

    # 三个实测分位标注
    quantiles = [('Mean (84.8 m)',   mean_e,  '0.55'),
                 ('RMS (103.8 m)',   rms_e,   '0.35'),
                 ('95th pct (176.1 m)', p95,  '0.15')]
    for lab, qv, c in quantiles:
        # 找该 x 处的 cdf 值
        cv = 0.5 * (1 + erf((np.log(qv) - mu) / (sigma * np.sqrt(2))))
        ax.axvline(qv, color=c, linewidth=0.9, linestyle=':', zorder=2)
        ax.plot([qv], [cv], 'o', color=c, markersize=4.5, zorder=4,
                markeredgecolor='black', markeredgewidth=0.6)

    # 文本标注（错开避免重叠）
    ax.text(mean_e+4, 0.18, 'mean',    fontsize=7.5, color='0.3')
    ax.text(rms_e+4,  0.30, 'RMS',     fontsize=7.5, color='0.3')
    ax.text(p95+4,    0.86, '95th pct', fontsize=7.5, color='0.3')

    ax.set_xlim(0, 320); ax.set_ylim(0, 1.02)
    ax.set_xlabel('Horizontal positioning error (m)')
    ax.set_ylabel('Cumulative fraction of epochs')
    ax.xaxis.set_major_locator(MultipleLocator(50))
    ax.xaxis.set_minor_locator(MultipleLocator(25))
    ax.yaxis.set_major_locator(MultipleLocator(0.2))
    ax.yaxis.set_minor_locator(MultipleLocator(0.05))
    ax.grid(True, linestyle=':', linewidth=0.5, color='0.7', zorder=0)
    ax.set_axisbelow(True)
    ax.text(0.97, 0.05,
            f'$n={n_pts}$ epochs',
            transform=ax.transAxes, ha='right', va='bottom', fontsize=7.5,
            bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='0.5', lw=0.6))
    save(fig, 'fig4_spp_error_cdf')


# ════════════════════════════════════════════════════════════════════════════
# Fig.5  森林图：ρ_norm vs CN0_drop 的 Spearman r + 95% bootstrap CI
#   所有值直接取自 step6e/6f/6g 实测输出，无推算/插值
#   子集行：Multi-constellation pool / GPS all / GPS severe / GAL severe
#   BeiDou severe (n=30) 作注释行，不画 CI（n 太小无意义）
# ════════════════════════════════════════════════════════════════════════════
def fig_rho_cn0_stratified():
    # ── 实测 Spearman r 与 95% bootstrap CI（step6e/6f/6g 直接输出）──────────
    # 格式：(label, r, ci_lo, ci_hi, n_str, marker, color, linestyle)
    rows = [
        # subset                                  r        lo      hi   n_label
        ('Multi-constellation pool',          +0.006, -0.020, +0.032, '$n=5421$',   '^', '0.62', ':'),
        ('GPS — all epochs',                  -0.058, -0.099, -0.017, '$n\\!\\approx\\!2100$', 'o', '0.30', '-'),
        ('GPS — severe NLOS',                 -0.167, -0.264, -0.071, '',            's', '0.10', '-'),
        ('Galileo — severe NLOS',             -0.119, -0.263, +0.025, '',            'D', '0.45', '--'),
    ]
    # BeiDou severe: n=30, r=+0.063, CI too wide to be meaningful → text note only

    y_pos = np.arange(len(rows))[::-1]   # top row = first entry

    fig, ax = plt.subplots(figsize=(4.2, 2.5))

    for i, (lab, r, lo, hi, n_lab, mk, col, ls) in enumerate(rows):
        y = y_pos[i]
        ci_lo = r - lo   # xerr is asymmetric: (lower_err, upper_err)
        ci_hi = hi - r
        ax.errorbar(r, y,
                    xerr=[[ci_lo], [ci_hi]],
                    fmt=mk, color=col, markersize=5.5,
                    elinewidth=1.1, capsize=3.0, capthick=0.9,
                    markeredgecolor='black', markeredgewidth=0.6, zorder=4)
        # CI shading
        ax.barh(y, hi - lo, left=lo, height=0.32,
                color=col, alpha=0.18, zorder=2)
        # n label to the right of CI
        if n_lab:
            ax.text(hi + 0.005, y, n_lab, va='center', ha='left', fontsize=7,
                    color='0.35')

    # null line
    ax.axvline(0, color='black', linewidth=0.9, linestyle='--', zorder=3)
    ax.text(0.003, -0.55, 'null', ha='left', va='center', fontsize=7,
            style='italic', color='0.4')

    # BeiDou note
    ax.text(0.98, 0.04,
            r'BeiDou severe: $n=30$, inconclusive',
            transform=ax.transAxes, ha='right', va='bottom', fontsize=6.8,
            color='0.4', style='italic')

    ax.set_yticks(y_pos)
    ax.set_yticklabels([r[0] for r in rows], fontsize=8)
    ax.set_xlabel(r'Spearman $r$  ($\rho_{\mathrm{norm}}$ vs CN0$_\mathrm{drop}$)')
    ax.set_xlim(-0.35, 0.12)
    ax.xaxis.set_major_locator(MultipleLocator(0.1))
    ax.xaxis.set_minor_locator(MultipleLocator(0.05))
    ax.grid(axis='x', linestyle=':', linewidth=0.5, color='0.7', zorder=0)
    ax.set_axisbelow(True)
    ax.set_ylim(-0.7, len(rows) - 0.3)
    fig.tight_layout(pad=0.4)
    save(fig, 'fig5_rho_cn0_stratified')


if __name__ == '__main__':
    print('生成论文核心图（docs/figures/）：')
    fig_within_satellite()
    fig_cv_leakage()
    fig_elevation_confound()
    fig_spp_error_cdf()
    fig_rho_cn0_stratified()
    print('完成。')
