#!/usr/bin/env python3
"""
generate_project_report.py
Generate a readable project report (Word) with embedded charts.
Run: python3 generate_project_report.py --out project_report.docx
"""
import argparse, io, datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

ap = argparse.ArgumentParser()
ap.add_argument('--out', default='project_report.docx')
args = ap.parse_args()

doc = Document()
sec = doc.sections[0]
sec.page_width    = Cm(21.0);  sec.page_height   = Cm(29.7)
sec.left_margin   = Cm(2.5);   sec.right_margin  = Cm(2.5)
sec.top_margin    = Cm(2.5);   sec.bottom_margin = Cm(2.5)

# ── Colour palette ────────────────────────────────────────────────────
C_BLUE   = (31,  97, 141)
C_GREEN  = (39, 174,  96)
C_RED    = (192,  57,  43)
C_ORANGE = (211, 84,   0)
C_GREY   = (127, 140, 141)
C_DARK   = ( 44,  62,  80)

# ── Style helpers ─────────────────────────────────────────────────────
def sf(run, name='Calibri', size=11, bold=False, italic=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run.bold = bold;  run.italic = italic
    if color: run.font.color.rgb = RGBColor(*color)

def heading(text, level=1, size=14, color=C_DARK, bold=True, space_before=16):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(space_before)
    p.paragraph_format.space_after  = Pt(4)
    r = p.add_run(text);  sf(r, size=size, bold=bold, color=color)
    return p

def subheading(text, color=C_BLUE):
    return heading(text, size=12, color=color, space_before=10)

def body(text, indent=0, size=11, space_after=5, italic=False):
    p = doc.add_paragraph()
    p.paragraph_format.space_after  = Pt(space_after)
    p.paragraph_format.space_before = Pt(0)
    if indent: p.paragraph_format.left_indent = Cm(indent)
    r = p.add_run(text);  sf(r, size=size, italic=italic)
    return p

def bullet(text, indent=0.8):
    body('• ' + text, indent=indent, space_after=3)

def highlight_box(label, value, unit='', sub='', color=C_BLUE):
    """Single-cell table as a coloured highlight card."""
    tbl = doc.add_table(rows=1, cols=1)
    tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    cell = tbl.rows[0].cells[0]
    cell.paragraphs[0].clear()
    cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    tc = cell._tc
    tcPr = tc.get_or_add_tcPr()
    shd = OxmlElement('w:shd')
    hex_color = '{:02X}{:02X}{:02X}'.format(*color)
    shd.set(qn('w:fill'), hex_color);  shd.set(qn('w:val'), 'clear')
    tcPr.append(shd)
    r1 = cell.paragraphs[0].add_run(f'{value}{unit}\n')
    sf(r1, size=20, bold=True, color=(255,255,255))
    r2 = cell.paragraphs[0].add_run(label)
    sf(r2, size=9, color=(230,230,230))
    if sub:
        r3 = cell.paragraphs[0].add_run('\n' + sub)
        sf(r3, size=8, italic=True, color=(210,210,210))
    doc.add_paragraph().paragraph_format.space_after = Pt(4)

def add_table(headers, rows, caption=''):
    if caption:
        cp = doc.add_paragraph()
        cp.paragraph_format.space_before = Pt(8)
        r = cp.add_run(caption);  sf(r, size=10, bold=True)
    tbl = doc.add_table(rows=1+len(rows), cols=len(headers))
    tbl.style = 'Table Grid';  tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, h in enumerate(headers):
        cell = tbl.rows[0].cells[i];  cell.paragraphs[0].clear()
        r = cell.paragraphs[0].add_run(h);  sf(r, size=9.5, bold=True)
        cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
        tc = cell._tc;  tcPr = tc.get_or_add_tcPr()
        shd = OxmlElement('w:shd')
        shd.set(qn('w:fill'), '{:02X}{:02X}{:02X}'.format(*C_BLUE))
        shd.set(qn('w:val'), 'clear');  tcPr.append(shd)
        r.font.color.rgb = RGBColor(255,255,255)
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = tbl.rows[ri+1].cells[ci];  cell.paragraphs[0].clear()
            r = cell.paragraphs[0].add_run(str(val));  sf(r, size=9.5)
            cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
            if ri % 2 == 1:
                tc = cell._tc;  tcPr = tc.get_or_add_tcPr()
                shd = OxmlElement('w:shd')
                shd.set(qn('w:fill'), 'EBF5FB');  shd.set(qn('w:val'), 'clear')
                tcPr.append(shd)
    doc.add_paragraph()

def embed_fig(fig, width_cm=15, caption=''):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=150, bbox_inches='tight')
    buf.seek(0)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run()
    run.add_picture(buf, width=Cm(width_cm))
    plt.close(fig);  buf.close()
    if caption:
        cp = doc.add_paragraph(caption)
        cp.alignment = WD_ALIGN_PARAGRAPH.CENTER
        cp.paragraph_format.space_after = Pt(8)
        for r in cp.runs: sf(r, size=9, italic=True, color=C_GREY)

# ══════════════════════════════════════════════════════════════════════
# CHART FUNCTIONS
# ══════════════════════════════════════════════════════════════════════

def chart_pipeline():
    """Pipeline block diagram."""
    fig, ax = plt.subplots(figsize=(14, 3.5))
    ax.set_xlim(0, 14);  ax.set_ylim(0, 3.5);  ax.axis('off')
    steps = [
        ('Step 2\nLiDAR Map\nBuild',     '#2E86C1', 0.8),
        ('Step 3\nSat. Geometry\n(Az/El)',  '#1A5276', 2.4),
        ('Step 4\nRay-Cast\nNLOS Det.',  '#117A65', 4.0),
        ('Step 5\nNLOS Label\nCompare',  '#117A65', 5.6),
        ('Step 6\nReflect.\nModel ΔL',   '#6C3483', 7.2),
        ('Step 7A\nGPS-only\nSPP WLS',   '#B7950B', 8.8),
        ('Step 7B\nMulti-GNSS\nSPP WLS', '#784212', 10.4),
    ]
    for label, color, x in steps:
        rect = mpatches.FancyBboxPatch((x, 0.9), 1.2, 1.6,
            boxstyle='round,pad=0.1', facecolor=color, edgecolor='white', linewidth=1.5)
        ax.add_patch(rect)
        ax.text(x+0.6, 1.7, label, ha='center', va='center',
                fontsize=7.5, color='white', fontweight='bold',
                multialignment='center')
        if x < 10.4:
            ax.annotate('', xy=(x+1.4, 1.7), xytext=(x+1.2, 1.7),
                arrowprops=dict(arrowstyle='->', color='#566573', lw=1.5))

    # Input labels
    ax.text(0.3, 3.0, 'LiDAR bag', fontsize=8, color='#2E86C1', style='italic')
    ax.text(1.8, 3.0, 'RINEX obs + nav\n+ INSPVAX traj', fontsize=8,
            color='#1A5276', style='italic', ha='center')
    ax.text(7.8, 3.0, 'NLOS labels\n+ ΔL estimates', fontsize=8,
            color='#6C3483', style='italic', ha='center')
    # Output labels
    ax.text(0.8+0.6, 0.4, '8.53M pts\n0.2m voxel', fontsize=7,
            ha='center', color='#2E86C1')
    ax.text(4.0+0.6, 0.4, 'F1=0.574\nPrec=0.454', fontsize=7,
            ha='center', color='#117A65')
    ax.text(7.2+0.6, 0.4, 'ΔL mean\n6.70m', fontsize=7,
            ha='center', color='#6C3483')
    ax.text(8.8+0.6, 0.4, '75.6m\nmean', fontsize=7,
            ha='center', color='#B7950B')
    ax.text(10.4+0.6, 0.4, '26.5m\nmean', fontsize=7,
            ha='center', color='#784212')
    ax.set_title('LiDAR–GNSS NLOS Pipeline  (7 Steps)', fontsize=11,
                 fontweight='bold', pad=8, color='#2C3E50')
    fig.tight_layout()
    return fig

def chart_spp_comparison():
    """Main result: GPS-only vs Multi-GNSS, three modes."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # Left: mean horizontal error
    modes   = ['Baseline', 'Exclusion', 'ΔL Correction']
    gps     = [75.6,  238.9, 92.3]
    multi   = [26.45,  99.0, 39.34]
    x = np.arange(3);  w = 0.35

    ax = axes[0]
    b1 = ax.bar(x - w/2, gps,   w, label='GPS-only  (Exp A)',
                color='#2980B9', alpha=0.85)
    b2 = ax.bar(x + w/2, multi, w, label='Multi-GNSS (Exp B)',
                color='#27AE60', alpha=0.85)
    ax.set_xticks(x);  ax.set_xticklabels(modes, fontsize=10)
    ax.set_ylabel('Mean Horizontal Error (m)', fontsize=10)
    ax.set_title('Mean Horizontal Error — All Modes', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9);  ax.grid(axis='y', alpha=0.3)
    ax.set_ylim(0, 280)
    for bar in b1:
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+4,
                f'{bar.get_height():.0f}', ha='center', va='bottom', fontsize=8.5,
                color='#2980B9', fontweight='bold')
    for bar in b2:
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+4,
                f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=8.5,
                color='#27AE60', fontweight='bold')
    # Improvement annotation on baseline
    ax.annotate('', xy=(0+w/2, 26.45), xytext=(0+w/2, 75.6),
                arrowprops=dict(arrowstyle='<->', color='#E74C3C', lw=1.8))
    ax.text(0+w/2+0.12, 50, '↓65%', color='#E74C3C', fontsize=9, fontweight='bold')

    # Right: RMS
    gps_rms   = [204.3, 655.9, 205.6]
    multi_rms = [30.85, 240.4,  49.61]
    b3 = axes[1].bar(x - w/2, gps_rms,   w, label='GPS-only  (Exp A)',
                     color='#2980B9', alpha=0.85)
    b4 = axes[1].bar(x + w/2, multi_rms, w, label='Multi-GNSS (Exp B)',
                     color='#27AE60', alpha=0.85)
    axes[1].set_xticks(x);  axes[1].set_xticklabels(modes, fontsize=10)
    axes[1].set_ylabel('RMS Horizontal Error (m)', fontsize=10)
    axes[1].set_title('RMS Horizontal Error — All Modes', fontsize=11, fontweight='bold')
    axes[1].legend(fontsize=9);  axes[1].grid(axis='y', alpha=0.3)
    axes[1].set_ylim(0, 750)
    for bar in b3:
        axes[1].text(bar.get_x()+bar.get_width()/2, bar.get_height()+6,
                f'{bar.get_height():.0f}', ha='center', va='bottom', fontsize=8,
                color='#2980B9', fontweight='bold')
    for bar in b4:
        axes[1].text(bar.get_x()+bar.get_width()/2, bar.get_height()+6,
                f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=8,
                color='#27AE60', fontweight='bold')
    fig.suptitle('Experiment A (GPS-only) vs Experiment B (GPS+BDS+Galileo)',
                 fontsize=12, fontweight='bold', y=1.01)
    fig.tight_layout()
    return fig

def chart_satellite_counts():
    """Satellite count comparison and NLOS breakdown."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    # Left: per-constellation bar
    ax = axes[0]
    constellations = ['GPS', 'BeiDou', 'Galileo', 'Total']
    gps_counts   = [6.3, 0,   0,   6.3]
    multi_counts = [5.8, 6.4, 3.8, 15.9]
    x = np.arange(4);  w = 0.35
    ax.bar(x - w/2, gps_counts,   w, label='GPS-only',   color='#2980B9', alpha=0.85)
    ax.bar(x + w/2, multi_counts, w, label='Multi-GNSS', color='#27AE60', alpha=0.85)
    ax.set_xticks(x);  ax.set_xticklabels(constellations)
    ax.set_ylabel('Satellites per Epoch (avg)', fontsize=10)
    ax.set_title('Average Visible Satellites per Epoch', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9);  ax.grid(axis='y', alpha=0.3)
    for i, (g, m) in enumerate(zip(gps_counts, multi_counts)):
        if g > 0:
            ax.text(i - w/2, g + 0.15, f'{g:.1f}', ha='center', fontsize=8.5,
                    color='#2980B9', fontweight='bold')
        ax.text(i + w/2, m + 0.15, f'{m:.1f}', ha='center', fontsize=8.5,
                color='#27AE60', fontweight='bold')

    # Right: NLOS/LOS breakdown for multi-GNSS
    ax2 = axes[1]
    labels = ['LOS\n(all constellations)', 'NLOS\n(all constellations)']
    sizes  = [6.4, 9.5]
    colors = ['#27AE60', '#E74C3C']
    explode = (0.05, 0)
    wedges, texts, autotexts = ax2.pie(sizes, explode=explode, labels=labels,
        colors=colors, autopct='%1.0f%%', startangle=90,
        textprops={'fontsize': 10})
    for at in autotexts: at.set_fontsize(11); at.set_fontweight('bold')
    ax2.set_title('Multi-GNSS: LOS vs NLOS Split\n(avg 15.9 sats/epoch)',
                  fontsize=11, fontweight='bold')
    ax2.text(0, -1.4, '6.4 LOS  +  9.5 NLOS  =  15.9 total/epoch',
             ha='center', fontsize=9, style='italic', color='#566573')
    fig.tight_layout()
    return fig

def chart_nlos_detection():
    """NLOS detection confusion matrix + precision/recall bar."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    # Left: confusion matrix heatmap
    ax = axes[0]
    cm = np.array([[1091, 307], [1312, 633]])
    labels_row = ['LiDAR\nNLOS', 'LiDAR\nLOS']
    labels_col = ['del2 NLOS', 'del2 LOS']
    im = ax.imshow(cm, cmap='Blues')
    ax.set_xticks([0,1]);  ax.set_xticklabels(labels_col, fontsize=10)
    ax.set_yticks([0,1]);  ax.set_yticklabels(labels_row, fontsize=10)
    ax.set_xlabel('Reference Labels (del2AINLOS ML)', fontsize=10)
    ax.set_ylabel('Our LiDAR Detector', fontsize=10)
    ax.set_title('NLOS Detection Confusion Matrix', fontsize=11, fontweight='bold')
    cell_labels = [['TP=1091\n(both NLOS)', 'FN=307\n(missed NLOS)'],
                   ['FP=1312\n(over-detected)', 'TN=633\n(both LOS)']]
    cell_colors = [['#1A5276', '#7FB3D3'], ['#B03A2E', '#27AE60']]
    for i in range(2):
        for j in range(2):
            ax.text(j, i, cell_labels[i][j], ha='center', va='center',
                    fontsize=9, fontweight='bold',
                    color='white' if cm[i,j] > 700 else 'white')
    plt.colorbar(im, ax=ax, shrink=0.8)

    # Right: Precision/Recall/F1 bar
    ax2 = axes[1]
    metrics = ['Precision', 'Recall', 'F1-score']
    values  = [0.454, 0.780, 0.574]
    colors_bar = ['#E74C3C', '#27AE60', '#F39C12']
    bars = ax2.barh(metrics, values, color=colors_bar, alpha=0.85, height=0.5)
    ax2.set_xlim(0, 1.1)
    ax2.axvline(0.5, color='#BDC3C7', linestyle='--', lw=1.5, label='50% threshold')
    ax2.axvline(0.7, color='#E74C3C', linestyle=':', lw=1.5, alpha=0.7,
                label='Target Precision (70%)')
    for bar, val in zip(bars, values):
        ax2.text(val + 0.02, bar.get_y() + bar.get_height()/2,
                f'{val:.3f}', va='center', fontsize=11, fontweight='bold')
    ax2.set_xlabel('Score', fontsize=10)
    ax2.set_title('LiDAR NLOS Detection Performance\n(vs del2AINLOS reference)',
                  fontsize=11, fontweight='bold')
    ax2.legend(fontsize=8.5)
    ax2.annotate('Correction fails\nbecause Precision < 50%',
                 xy=(0.454, 0), xytext=(0.3, -0.55),
                 fontsize=8.5, color='#E74C3C', fontweight='bold',
                 arrowprops=dict(arrowstyle='->', color='#E74C3C'))
    ax2.grid(axis='x', alpha=0.3)
    fig.tight_layout()
    return fig

def chart_delta_l():
    """ΔL severity distribution + mean comparison."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    # Left: GPS-only donut
    sizes  = [14.6, 67.9, 17.5]
    labels = ['Mild\n<2m\n14.6%', 'Strong\n2–10m\n67.9%', 'Severe\n≥10m\n17.5%']
    colors = ['#F7DC6F', '#E67E22', '#E74C3C']
    wedges, texts = axes[0].pie(sizes, labels=labels, colors=colors,
        startangle=90, wedgeprops=dict(width=0.55),
        textprops={'fontsize': 9})
    axes[0].set_title('GPS-only ΔL Severity (2753 NLOS)\nMean = 6.70 m',
                      fontsize=11, fontweight='bold')

    # Right: Multi-GNSS donut
    sizes2  = [13.2, 69.8, 17.0]
    labels2 = ['Mild\n<2m\n13.2%', 'Strong\n2–10m\n69.8%', 'Severe\n≥10m\n17.0%']
    wedges2, texts2 = axes[1].pie(sizes2, labels=labels2, colors=colors,
        startangle=90, wedgeprops=dict(width=0.55),
        textprops={'fontsize': 9})
    axes[1].set_title('Multi-GNSS ΔL Severity (6578 NLOS)\nMean = 6.69 m',
                      fontsize=11, fontweight='bold')
    fig.suptitle('Extra Path Length (ΔL) Distribution — Both Experiments',
                 fontsize=12, fontweight='bold')
    fig.tight_layout()
    return fig

def chart_why_correction_fails():
    """Annotated diagram explaining why correction degrades accuracy."""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlim(0, 12);  ax.set_ylim(0, 5);  ax.axis('off')

    # Title
    ax.text(6, 4.7, 'Why Does ΔL Correction Make Things Worse?',
            ha='center', va='center', fontsize=13, fontweight='bold', color='#2C3E50')

    # Four cause boxes
    causes = [
        ('① Detector\nPrecision 45.4%',
         '55% of "NLOS" labels are\nactually LOS or partial NLOS.\nCorrection applied to\ngood pseudoranges.', '#E74C3C', 0.5),
        ('② Mixed\nMultipath',
         'For partial NLOS:\nDLL error = ΔL × f(A_r/A_d)\nAmplitude ratio unknown\n→ overcorrection', '#E67E22', 3.3),
        ('③ Low Planarity\n44.9%',
         'Nearly half of surface\nnormals are unreliable\n(edges, vegetation, clutter)\n→ wrong ΔL estimates', '#8E44AD', 6.1),
        ('④ Weight\nSuppression',
         'w_NLOS = sin²(el) × planarity × 0.1\nCorrected sats contribute\nonly ~5% to WLS\n→ correction diluted', '#2980B9', 8.9),
    ]
    for title, desc, color, x in causes:
        rect = mpatches.FancyBboxPatch((x, 1.5), 2.6, 2.8,
            boxstyle='round,pad=0.15', facecolor=color, alpha=0.15,
            edgecolor=color, linewidth=2)
        ax.add_patch(rect)
        ax.text(x+1.3, 3.8, title, ha='center', va='center',
                fontsize=9.5, fontweight='bold', color=color)
        ax.text(x+1.3, 2.4, desc, ha='center', va='center',
                fontsize=8, color='#2C3E50', multialignment='center')

    # Bottom result box
    result = mpatches.FancyBboxPatch((3.5, 0.2), 5.0, 0.9,
        boxstyle='round,pad=0.1', facecolor='#E74C3C', alpha=0.9, edgecolor='white')
    ax.add_patch(result)
    ax.text(6, 0.65, 'Result: correction mean 39.3 m  >  baseline 26.5 m  (+12.9 m worse)',
            ha='center', va='center', fontsize=9.5, fontweight='bold', color='white')

    # Arrows from boxes to result
    for x in [1.8, 4.6, 7.4, 10.2]:
        ax.annotate('', xy=(6, 1.1), xytext=(x, 1.5),
                    arrowprops=dict(arrowstyle='->', color='#7F8C8D', lw=1.2))
    fig.tight_layout()
    return fig

def chart_comparison_with_others():
    """Comparison of our approach vs Wen 2019 vs PPT student."""
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.set_xlim(0, 12);  ax.set_ylim(0, 4);  ax.axis('off')

    works = [
        ('Wen et al.\n2019',
         ['• Multi-constellation (GPS+BDS+GLO+GAL)',
          '• Real-time 250m sliding-window map',
          '• Single ray + building height filter',
          '• Precision estimated >80%'],
         '~6 m RMS\n(4× improvement)', '#27AE60', 0.3),
        ('PPT Student\n(2024)',
         ['• GPS + BDS constellation',
          '• Open3DHK city model (clean)',
          '• Multi-candidate ray selection',
          '• Improvement in UP direction'],
         'UP direction\nimproved', '#F39C12', 4.3),
        ('Our Work\n(2024)',
         ['• GPS + BDS + Galileo',
          '• Offline accumulated LiDAR map',
          '• Single ray, first intersection',
          '• Baseline 26.5m (↓65% vs GPS-only)'],
         'Baseline ↓65%\nΔL correction\nstill fails', '#2980B9', 8.3),
    ]
    for title, points, result, color, x in works:
        rect = mpatches.FancyBboxPatch((x, 0.3), 3.5, 3.2,
            boxstyle='round,pad=0.15', facecolor=color, alpha=0.1,
            edgecolor=color, linewidth=2)
        ax.add_patch(rect)
        ax.text(x+1.75, 3.3, title, ha='center', va='center',
                fontsize=10, fontweight='bold', color=color)
        for i, pt in enumerate(points):
            ax.text(x+0.2, 2.6-i*0.5, pt, va='center',
                    fontsize=8, color='#2C3E50')
        # Result badge
        badge = mpatches.FancyBboxPatch((x+0.4, 0.35), 2.7, 0.7,
            boxstyle='round,pad=0.08', facecolor=color, alpha=0.8)
        ax.add_patch(badge)
        ax.text(x+1.75, 0.7, result, ha='center', va='center',
                fontsize=8.5, fontweight='bold', color='white', multialignment='center')
    ax.set_title('Comparison with Related Work', fontsize=12,
                 fontweight='bold', pad=5, color='#2C3E50', y=0.98)
    ax.text(6, 0.05, 'Key insight: map quality and detector precision determine correction effectiveness',
            ha='center', fontsize=9, style='italic', color='#7F8C8D')
    fig.tight_layout()
    return fig

def chart_improvement_summary():
    """Summary of our multi-constellation improvement."""
    fig, ax = plt.subplots(figsize=(10, 4))

    categories = ['Mean Error (m)', 'RMS Error (m)', '50th pct (m)', '95th pct (m)',
                  'Valid Excl. Epochs']
    gps_vals   = [75.6,  204.3, 47.2,  116.8,  85]
    multi_vals = [26.45, 30.85, 22.39, 56.67, 359]

    x = np.arange(len(categories));  w = 0.35
    bars1 = ax.bar(x - w/2, gps_vals,   w, label='GPS-only',   color='#2980B9', alpha=0.85)
    bars2 = ax.bar(x + w/2, multi_vals, w, label='Multi-GNSS', color='#27AE60', alpha=0.85)

    ax.set_xticks(x);  ax.set_xticklabels(categories, fontsize=9)
    ax.set_ylabel('Value', fontsize=10)
    ax.set_title('Baseline Mode: GPS-only vs Multi-GNSS — Full Comparison',
                 fontsize=11, fontweight='bold')
    ax.legend(fontsize=10);  ax.grid(axis='y', alpha=0.3)

    for bar1, bar2 in zip(bars1, bars2):
        v1, v2 = bar1.get_height(), bar2.get_height()
        ax.text(bar1.get_x()+bar1.get_width()/2, v1+2, f'{v1:.0f}',
                ha='center', fontsize=8, color='#2980B9', fontweight='bold')
        ax.text(bar2.get_x()+bar2.get_width()/2, v2+2, f'{v2:.0f}',
                ha='center', fontsize=8, color='#27AE60', fontweight='bold')
        pct = (v1 - v2) / v1 * 100
        mid_x = (bar1.get_x() + bar2.get_x() + bar2.get_width()) / 2
        if v1 > 0:
            ax.text(mid_x, max(v1, v2) * 0.6,
                    f'↓{pct:.0f}%' if pct > 0 else f'↑{-pct:.0f}%',
                    ha='center', fontsize=8.5,
                    color='#E74C3C' if pct > 0 else '#27AE60', fontweight='bold')
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════
# DOCUMENT CONTENT
# ══════════════════════════════════════════════════════════════════════

# ── Cover page ────────────────────────────────────────────────────────
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
p.paragraph_format.space_before = Pt(40)
r = p.add_run('LiDAR-GNSS NLOS Pipeline\nProject Report')
sf(r, size=22, bold=True, color=C_BLUE)

p2 = doc.add_paragraph()
p2.alignment = WD_ALIGN_PARAGRAPH.CENTER
r2 = p2.add_run(
    'Urban Canyon GNSS NLOS Detection, Reflection Modeling,\n'
    'and Multi-Constellation SPP Correction\n\n'
    'UrbanNav Medium-Urban-1  ·  Hong Kong TST  ·  2021-05-17')
sf(r2, size=12, color=C_GREY, italic=True)

p3 = doc.add_paragraph()
p3.alignment = WD_ALIGN_PARAGRAPH.CENTER
r3 = p3.add_run(datetime.datetime.now().strftime('%B %Y'))
sf(r3, size=11, color=C_GREY)
doc.add_page_break()

# ── Executive Summary ─────────────────────────────────────────────────
heading('Executive Summary')
body(
    'This report describes a complete 7-step pipeline that uses LiDAR point clouds '
    'to detect GNSS non-line-of-sight (NLOS) signals and estimate pseudorange corrections '
    'in urban canyon environments. The pipeline was evaluated on the publicly available '
    'UrbanNav Medium-Urban-1 dataset collected in Hong Kong\'s Tsim Sha Tsui district.'
)
body(
    'Two experiments were conducted: Experiment A used GPS-only (9 satellites/epoch), '
    'and Experiment B added BeiDou and Galileo (16 satellites/epoch). '
    'The key finding is that multi-constellation geometry is the critical prerequisite '
    'for NLOS correction to be effective — adding BeiDou and Galileo alone reduced '
    'the baseline SPP error by 65% (75.6 m → 26.5 m) without any correction applied.'
)

# Key metrics row
tbl_sum = doc.add_table(rows=1, cols=4)
tbl_sum.alignment = WD_TABLE_ALIGNMENT.CENTER
metrics_sum = [
    ('26.5 m', 'Multi-GNSS Baseline\nMean Horizontal Error', (39,174,96)),
    ('65%', 'Improvement over\nGPS-only Baseline',       (31,97,141)),
    ('0.574', 'NLOS Detection\nF1-score',                  (142,68,173)),
    ('6.69 m', 'Mean Extra Path\nΔL (6578 NLOS)',          (211,84,0)),
]
for i, (val, lbl, col) in enumerate(metrics_sum):
    cell = tbl_sum.rows[0].cells[i]
    cell.paragraphs[0].clear()
    cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    tc = cell._tc;  tcPr = tc.get_or_add_tcPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:fill'), '{:02X}{:02X}{:02X}'.format(*col))
    shd.set(qn('w:val'), 'clear');  tcPr.append(shd)
    r1 = cell.paragraphs[0].add_run(val + '\n')
    sf(r1, size=18, bold=True, color=(255,255,255))
    r2 = cell.paragraphs[0].add_run(lbl)
    sf(r2, size=8.5, color=(220,220,220))
doc.add_paragraph()

# ── Section 1: Background ─────────────────────────────────────────────
heading('1.  What Is This Project About?')
body(
    'In cities like Hong Kong, GPS signals bounce off tall buildings before reaching '
    'the receiver. These reflected (NLOS) signals make the measured distance to the '
    'satellite appear longer than it really is — sometimes by 10–40 metres — causing '
    'position errors of 50–200 metres for standard GPS-only positioning.'
)
body(
    'Our goal: use a LiDAR (laser scanner) point cloud map of the city to:'
)
bullet('Detect which satellite signals are blocked or reflected by buildings (NLOS detection)')
bullet('Estimate how much extra distance the reflected signal travelled (ΔL correction)')
bullet('Use this information to improve GPS positioning accuracy (WLS SPP)')
body(
    'The dataset: UrbanNav Medium-Urban-1, collected by a car driving through '
    'Tsim Sha Tsui, Hong Kong. The car has a u-blox F9P GNSS receiver, '
    'a Velodyne VLP-16 LiDAR, and a NovAtel SPAN RTK/INS system providing '
    '5 cm-accurate ground truth.'
)

# ── Section 2: Pipeline ───────────────────────────────────────────────
heading('2.  Our 7-Step Pipeline')
embed_fig(chart_pipeline(), width_cm=16,
          caption='Figure 1. Complete LiDAR–GNSS NLOS pipeline from raw data to SPP correction.')

subheading('Step 2 — Build LiDAR Map')
body(
    'We accumulated all LiDAR scans along the 13-minute drive into a single 3D map, '
    'applying the vehicle\'s precise RTK/INS position at each moment to correctly '
    'place each scan. The resulting map contains 8.53 million points and was '
    'downsampled to 0.2 m voxels for efficient processing. '
    'Ground points below 1.0 m height were removed to prevent vehicle-body reflections '
    'from interfering with building detection.'
)
bullet('Map size: 8.53 million points')
bullet('Voxel resolution: 0.2 m')
bullet('Ground filter: Z > 1.0 m')

subheading('Step 3 — Compute Satellite Positions')
body(
    'For each of the 657 GNSS observation epochs (one per second), we computed '
    'the azimuth (compass direction) and elevation angle of every visible satellite '
    'using the broadcast navigation messages from the CORS reference station (HKSC). '
    'In Experiment B, we loaded separate navigation files for GPS (.21n), '
    'BeiDou (.21f), and Galileo (.21l), tripling the number of satellites processed.'
)
bullet('Experiment A: GPS only — ~6.3 satellites/epoch, 4140 records total')
bullet('Experiment B: GPS + BeiDou + Galileo — ~15.9 satellites/epoch, 11050 records total')
bullet('Leap-second correction (18 s) applied for INSPVAX time matching')

subheading('Step 4 — Ray-Casting NLOS Detection')
body(
    'For each satellite, we cast a single ray from the receiver position toward '
    'the satellite through the 3D voxel occupancy grid. If the ray hits a building '
    'voxel within 80 m, the satellite is classified as NLOS; otherwise LOS. '
    'The intersection distance d_hit is recorded for use in Step 6.'
)
bullet('Ray start offset: 5.0 m (skip vehicle body)')
bullet('Maximum range: 80.0 m')
bullet('Step size: 0.5 m per increment')
bullet('Experiment B NLOS rate: 59.5% (6578 NLOS out of 11050)')

subheading('Step 5 — Compare with ML Reference Labels')
body(
    'We compared our LiDAR geometric labels against del2AINLOS, a machine-learning '
    'NLOS classifier built into the del2AINLOS ROS package. del2AINLOS uses '
    'double-difference pseudorange residuals to detect NLOS — a completely different '
    'approach from ours, making it a useful independent reference.'
)

subheading('Step 6 — Reflection Surface Modeling')
body(
    'For each NLOS satellite, we found the building surface at the ray intersection '
    'point by searching the 30 nearest LiDAR map points within 3 m and fitting '
    'a plane using PCA (Principal Component Analysis). The surface normal and '
    'incidence angle give us the extra path length: ΔL = 2 × d_hit × cos(θ).'
)
bullet('Planarity check: if planarity < 0.5, use simplified fallback ΔL = 2d·cos(elevation)')
bullet('Experiment B: 6578 NLOS processed, 44.9% had low planarity')
bullet('ΔL distribution: 13% mild, 70% strong (2–10 m), 17% severe (>10 m)')

subheading('Step 7 — WLS SPP Correction')
body(
    'We ran three positioning modes for comparison: baseline (all satellites, '
    'no correction), NLOS exclusion (remove LiDAR-NLOS satellites), and '
    'ΔL correction (apply extra path correction to NLOS pseudoranges).'
)
body(
    'For Experiment B (multi-constellation), the WLS solver was extended to '
    'estimate separate receiver clock biases for each constellation: '
    '[X, Y, Z, c·δt_GPS, c·δt_BDS, c·δt_GAL] — 6 unknowns instead of 4.'
)

# ── Section 3: Results ────────────────────────────────────────────────
heading('3.  Results')
subheading('3.1  NLOS Detection Performance')

embed_fig(chart_nlos_detection(), width_cm=15.5,
          caption='Figure 2. Left: NLOS detection confusion matrix vs del2AINLOS labels. '
                  'Right: Precision, Recall, and F1-score.')
body(
    'Our LiDAR detector achieves high Recall (0.780) — it rarely misses a true NLOS '
    'satellite. However, Precision is only 0.454, meaning more than half of the '
    'satellites we label as NLOS are actually LOS or weakly affected. '
    'This over-detection is caused by the offline accumulated map: distant buildings '
    'that the car passed earlier appear in the map but do not actually block current signals, '
    'causing false NLOS predictions especially for high-elevation satellites (>45°).'
)
add_table(
    ['Metric', 'Value', 'Meaning'],
    [
        ['True Positives (TP)',  '1091', 'Correctly detected NLOS'],
        ['False Positives (FP)', '1312', 'LOS satellite wrongly flagged as NLOS'],
        ['False Negatives (FN)', '307',  'NLOS satellite missed by LiDAR'],
        ['True Negatives (TN)',  '633',  'Correctly identified LOS'],
        ['Precision',           '0.454', '45.4% of our NLOS labels are correct'],
        ['Recall',              '0.780', '78% of true NLOS are caught'],
        ['F1-score',            '0.574', 'Harmonic mean of Precision & Recall'],
    ],
    caption='Table 1. NLOS detection performance vs del2AINLOS ML reference labels.'
)

subheading('3.2  Reflection Geometry (ΔL Distribution)')
embed_fig(chart_delta_l(), width_cm=14,
          caption='Figure 3. Extra path length (ΔL) severity distribution for '
                  'GPS-only (left) and Multi-GNSS (right). Distribution is highly '
                  'consistent across both experiments.')
body(
    'The ΔL distribution is dominated by the "strong" category (2–10 m, ~70%), '
    'consistent with urban canyon multipath literature. The mean ΔL of ~6.7 m '
    'across both experiments shows that the geometry is similar regardless of '
    'constellation — BeiDou and Galileo experience the same building reflections '
    'as GPS in this environment.'
)

subheading('3.3  Satellite Availability')
embed_fig(chart_satellite_counts(), width_cm=14,
          caption='Figure 4. Left: average satellites per epoch by constellation. '
                  'Right: LOS/NLOS split for multi-GNSS experiment.')
body(
    'Adding BeiDou (6.4 sats/epoch) and Galileo (3.8 sats/epoch) to GPS (5.8 sats/epoch) '
    'raises the total to 15.9 satellites per epoch — a 2.5× increase. '
    'Even after the 59.5% NLOS exclusion, 6.4 LOS satellites remain per epoch on average, '
    'well above the minimum of 6 required for the 6-unknown multi-constellation WLS solver.'
)

subheading('3.4  SPP Positioning Results')
embed_fig(chart_spp_comparison(), width_cm=16,
          caption='Figure 5. SPP horizontal error comparison: GPS-only (Exp A) vs '
                  'Multi-GNSS (Exp B) for all three correction modes. '
                  'Note: exclusion RMS values are clipped for readability.')
embed_fig(chart_improvement_summary(), width_cm=15,
          caption='Figure 6. Baseline mode detailed comparison: all error metrics '
                  'across GPS-only and Multi-GNSS experiments.')
add_table(
    ['Experiment', 'Mode', 'Valid Epochs', 'Mean (m)', 'RMS (m)', '50th (m)', '95th (m)'],
    [
        ['Exp A: GPS-only',   'Baseline',    '635', '75.6',  '204.3', '47.2',  '116.8'],
        ['Exp A: GPS-only',   'Exclusion',    '85', '238.9', '655.9', '95.5',  '1154.4'],
        ['Exp A: GPS-only',   'ΔL Correction','635', '92.3', '205.6', '68.7',  '141.2'],
        ['Exp B: Multi-GNSS', 'Baseline',    '657', '26.5',  '30.9',  '22.4',  '56.7'],
        ['Exp B: Multi-GNSS', 'Exclusion',   '359', '99.0',  '240.4', '38.9',  '300.9'],
        ['Exp B: Multi-GNSS', 'ΔL Correction','657', '39.3', '49.6',  '33.1',  '100.1'],
    ],
    caption='Table 2. Complete SPP horizontal error statistics — both experiments, all modes.'
)

# ── Section 4: Why Correction Fails ──────────────────────────────────
heading('4.  Why Does ΔL Correction Not Work?')
embed_fig(chart_why_correction_fails(), width_cm=16,
          caption='Figure 7. Root-cause diagram: four compounding factors explain '
                  'why ΔL correction is worse than baseline in both experiments.')
body(
    'Despite having correct physics and real LiDAR geometry, the ΔL correction '
    'consistently makes positioning worse rather than better. Four factors compound:'
)
causes_text = [
    ('Detector Precision too low (45.4%)',
     'More than half our "NLOS" satellites are actually LOS. Subtracting ΔL '
     'from a good pseudorange corrupts it rather than fixing it. On average, '
     '~5 satellites per epoch are wrongly corrected, while only ~4 truly benefit.'),
    ('Geometric ΔL ≠ actual tracking error',
     'The formula ΔL = 2d·cos(θ) assumes pure NLOS (direct path completely blocked). '
     'For partial NLOS (both direct and reflected signals present), the DLL tracking '
     'error is δρ = ΔL × f(A_r/A_d), where A_r/A_d is the signal amplitude ratio '
     '— a quantity we cannot know without raw signal data (SDR measurement).'),
    ('Low surface normal reliability (44.9% planarity < 0.5)',
     'Building edges, parked vehicles, and tree branches in the accumulated map '
     'create "messy" geometry at the reflection points. Nearly half the ΔL estimates '
     'use the simplified fallback formula rather than the true PCA surface normal.'),
    ('Down-weighting suppresses the correction',
     'NLOS corrected satellites are assigned weight = sin²(el) × planarity × 0.1, '
     'giving them only ~5% of a normal satellite\'s contribution to WLS. '
     'Even when ΔL is accurate, the corrected pseudorange barely influences the solution.'),
]
for i, (title, desc) in enumerate(causes_text, 1):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent  = Cm(0.5)
    p.paragraph_format.space_before = Pt(5)
    r1 = p.add_run(f'({i})  {title}:  ')
    sf(r1, size=11, bold=True, color=C_BLUE)
    r2 = p.add_run(desc)
    sf(r2, size=11)

# ── Section 5: Comparison ─────────────────────────────────────────────
heading('5.  How Do We Compare with Related Work?')
embed_fig(chart_comparison_with_others(), width_cm=15.5,
          caption='Figure 8. Comparison of our pipeline with Wen et al. 2019 '
                  'and the PPT presentation (Point Cloud Aided GNSS Signal Path Tracking).')
add_table(
    ['', 'Wen et al. 2019', 'PPT Presentation', 'Our Work'],
    [
        ['Point cloud source',   'Real-time LiDAR\n(250m window)', 'Open3DHK city model', 'Offline accumulated\nLiDAR map'],
        ['Constellation',        'GPS+BDS+GLO+GAL',  'GPS+BDS',          'GPS+BDS+GAL'],
        ['Ray strategy',         'Single ray +\nbuilding height', 'Multi-candidate\n+ best selection', 'Single ray,\nfirst intersection'],
        ['NLOS Precision',       'Estimated >80%',   'Not reported',     '45.4%'],
        ['ΔL correction effect', '~4× improvement',  'UP dir. improved', 'Worse than baseline'],
        ['Key advantage',        'Real-time map\nelim. false NLOS', 'Clean geometry\n→ reliable ΔL', 'Full pipeline\n+ failure analysis'],
    ],
    caption='Table 3. Comparison with related work.'
)
body(
    'The key insight from this comparison: ΔL correction effectiveness is gated by '
    'NLOS detector precision, not by the ΔL formula itself. Wen 2019 and the PPT '
    'student achieve better precision through real-time or cleaner point clouds. '
    'Our offline accumulated map introduces clutter that reduces precision below 50%, '
    'inverting the correction effect.'
)

# ── Section 6: Advantages ─────────────────────────────────────────────
heading('6.  What We Did Well')
bullet('Complete, reproducible pipeline — all 7 steps documented and open-sourced on GitHub.')
bullet('Quantitative analysis at every step — not just final accuracy, '
       'but detection F1, ΔL distribution, planarity statistics.')
bullet('Honest negative result with root-cause analysis — we explain exactly '
       'why correction fails, which is more valuable than hiding a failure.')
bullet('Multi-constellation extension — extended GPS-only pipeline to GPS+BDS+GAL '
       'achieving a 65% accuracy improvement on the baseline alone.')
bullet('Real LiDAR point cloud — unlike Open3DHK, our map comes from the actual sensor, '
       'making the pipeline applicable to vehicles without pre-existing city models.')
bullet('Cross-validation — compared our geometric labels against an independent '
       'ML-based NLOS detector (del2AINLOS), providing dual-method validation.')

# ── Section 7: Limitations ────────────────────────────────────────────
heading('7.  Current Limitations and Next Steps')
subheading('Limitations')
bullet('Offline accumulated map introduces false NLOS for high-elevation satellites '
       '(F1 drops to 0.242 for el > 45°).')
bullet('Single-ray strategy cannot handle multi-bounce reflections or find the '
       'geometrically optimal reflection path.')
bullet('ΔL correction assumes pure NLOS; amplitude ratio for mixed multipath is unknown.')
bullet('GLONASS excluded (different orbital mechanics require separate implementation).')

subheading('Next Steps — In Priority Order')
steps_next = [
    ('Real-time sliding-window map (250 m)',
     'Replace offline map with a window following the vehicle. '
     'Expected to reduce false NLOS rate for high-elevation satellites '
     'and improve precision from 45% toward 80%+.'),
    ('SDR amplitude measurement',
     'Capture raw GNSS IF data with a USRP N210 SDR. Measure the '
     'reflected-to-direct amplitude ratio A_r/A_d to convert geometric '
     'ΔL into the actual DLL tracking error δρ.'),
    ('Multi-candidate ray selection',
     'Generate multiple candidate reflection paths per satellite and '
     'select the most geometrically plausible one (following the PPT approach). '
     'Reduces d_hit error at building edges.'),
    ('GLONASS integration',
     'Add numerical integration orbit propagation for GLONASS. '
     'With GLONASS, total constellation rises to ~20 satellites/epoch.'),
]
for title, desc in steps_next:
    p = doc.add_paragraph()
    p.paragraph_format.left_indent  = Cm(0.5)
    p.paragraph_format.space_before = Pt(6)
    r1 = p.add_run(f'▶  {title}:  ')
    sf(r1, size=11, bold=True, color=C_GREEN)
    r2 = p.add_run(desc)
    sf(r2, size=11)

# ── Section 8: Data summary ───────────────────────────────────────────
heading('8.  Full Experimental Data')
add_table(
    ['Parameter', 'Value'],
    [
        ['Dataset',            'UrbanNav Medium-Urban-1'],
        ['Location',           'Tsim Sha Tsui, Hong Kong'],
        ['Date / Duration',    '2021-05-17, ~13 minutes'],
        ['GNSS receiver',      'u-blox F9P (GPS/BDS/GAL/GLO capable)'],
        ['LiDAR sensor',       'Velodyne VLP-16 (10 Hz)'],
        ['Ground truth',       'NovAtel INSPVAX RTK/INS (~5 cm)'],
        ['LiDAR map points',   '8,531,577 (after 0.2 m downsampling)'],
        ['Exp A obs epochs',   '657  |  GPS satellites: 4140 records'],
        ['Exp B obs records',  '11050  (GPS 37.5%, BDS 38.0%, GAL 24.6%)'],
        ['NLOS rate (Exp B)',  '59.5%  (6578 / 11050)'],
        ['Mean ΔL',            '6.69 m  (range: 0–38 m)'],
        ['Nav files used',     'hksc137c.21n + .21f + .21l  (Hong Kong CORS)'],
    ],
    caption='Table 4. Dataset and pipeline parameters.'
)

doc.save(args.out)
print(f'Saved: {args.out}')
