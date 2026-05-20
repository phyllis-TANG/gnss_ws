#!/usr/bin/env python3
"""
generate_paper_draft.py
Generate paper draft Word document (python-docx)
Run: python3 generate_paper_draft.py --out paper_draft.docx
"""
import argparse
from docx import Document
from docx.shared import Pt, Cm, RGBColor, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
import datetime

ap = argparse.ArgumentParser()
ap.add_argument('--out', default='paper_draft.docx')
args = ap.parse_args()

doc = Document()

# ── Page setup ───────────────────────────────────────────────────────
section = doc.sections[0]
section.page_width  = Cm(21.0)
section.page_height = Cm(29.7)
section.left_margin   = Cm(2.5)
section.right_margin  = Cm(2.5)
section.top_margin    = Cm(2.5)
section.bottom_margin = Cm(2.5)

# ── Style helpers ────────────────────────────────────────────────────
def set_font(run, name='Times New Roman', size=11, bold=False, italic=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run.bold  = bold
    run.italic = italic
    if color:
        run.font.color.rgb = RGBColor(*color)

def heading(text, level=1, size=13, bold=True, color=None):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(12)
    p.paragraph_format.space_after  = Pt(4)
    run = p.add_run(text)
    set_font(run, size=size, bold=bold, color=color)
    return p

def body(text, indent=0, italic=False, size=11, space_after=6):
    p = doc.add_paragraph()
    p.paragraph_format.space_after  = Pt(space_after)
    p.paragraph_format.space_before = Pt(0)
    if indent:
        p.paragraph_format.left_indent = Cm(indent)
    run = p.add_run(text)
    set_font(run, size=size, italic=italic)
    return p

def note(text):
    """Grey draft annotation paragraph."""
    p = doc.add_paragraph()
    p.paragraph_format.left_indent  = Cm(1.0)
    p.paragraph_format.space_after  = Pt(4)
    run = p.add_run('[DRAFT NOTE] ' + text)
    set_font(run, size=9.5, italic=True, color=(120, 120, 120))
    return p

def add_table(headers, rows, caption=''):
    if caption:
        cp = doc.add_paragraph()
        cp.paragraph_format.space_before = Pt(8)
        r = cp.add_run(caption)
        set_font(r, size=10, bold=True)
    tbl = doc.add_table(rows=1+len(rows), cols=len(headers))
    tbl.style = 'Table Grid'
    tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    # Header row
    for i, h in enumerate(headers):
        cell = tbl.rows[0].cells[i]
        cell.paragraphs[0].clear()
        run = cell.paragraphs[0].add_run(h)
        set_font(run, size=10, bold=True)
        cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
        # Header background colour
        tc = cell._tc
        tcPr = tc.get_or_add_tcPr()
        shd = OxmlElement('w:shd')
        shd.set(qn('w:fill'), 'D6E4F0')
        shd.set(qn('w:val'), 'clear')
        tcPr.append(shd)
    # Data rows
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = tbl.rows[ri+1].cells[ci]
            cell.paragraphs[0].clear()
            run = cell.paragraphs[0].add_run(str(val))
            set_font(run, size=10)
            cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    doc.add_paragraph()
    return tbl

def divider():
    p = doc.add_paragraph('─' * 60)
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after  = Pt(2)
    for run in p.runs:
        set_font(run, size=8, color=(180,180,180))

# ══════════════════════════════════════════════════════════════════════
# Title page
# ══════════════════════════════════════════════════════════════════════
p_title = doc.add_paragraph()
p_title.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_title.paragraph_format.space_before = Pt(24)
p_title.paragraph_format.space_after  = Pt(8)
r = p_title.add_run(
    'LiDAR Point Cloud-Based GNSS NLOS Detection and\n'
    'Reflection Geometry Modeling in Urban Canyons:\n'
    'A Pipeline Evaluation with GPS-Only SPP')
set_font(r, size=16, bold=True)

p_sub = doc.add_paragraph()
p_sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_sub.paragraph_format.space_after = Pt(6)
r2 = p_sub.add_run(
    'LiDAR-Aided GNSS NLOS Geometric Modeling and\nPseudorange Correction Evaluation in Urban Canyons  (Draft v0.1)')
set_font(r2, size=12, italic=True, color=(80,80,80))

p_author = doc.add_paragraph()
p_author.alignment = WD_ALIGN_PARAGRAPH.CENTER
r3 = p_author.add_run('[Author Names] · [Affiliation] · ' +
                      datetime.datetime.now().strftime('%Y-%m-%d'))
set_font(r3, size=10, color=(100,100,100))

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════
# Abstract
# ══════════════════════════════════════════════════════════════════════
heading('Abstract', size=13)
body(
    'Urban canyon environments severely degrade GNSS positioning accuracy due to '
    'non-line-of-sight (NLOS) signal reception and multipath propagation. This paper '
    'presents an end-to-end pipeline that leverages LiDAR point cloud maps for GNSS '
    'NLOS detection, reflection surface modeling, and pseudorange correction, evaluated '
    'on the publicly available UrbanNav Medium-Urban-1 dataset collected in the Tsim '
    'Sha Tsui district of Hong Kong. '
    'The pipeline comprises five stages: (1) INSPVAX trajectory extraction, '
    '(2) dense LiDAR map construction (8.53 M points, 0.2 m voxel), '
    '(3) satellite azimuth/elevation computation from RINEX observations, '
    '(4) ray-casting-based NLOS detection via a 3D occupancy voxel grid, and '
    '(5) reflection surface geometry modeling using local PCA surface normals, '
    'yielding per-satellite extra path length estimates (ΔL = 2·d·cos θᵢ). '
    'Compared against del2AINLOS machine-learning labels, the LiDAR detector achieves '
    'Precision = 0.454, Recall = 0.780, and F1 = 0.574. '
    'The geometric ΔL follows a distribution dominated by the "strong" category '
    '(2–10 m, 67.9%), with a mean of 6.70 m consistent with urban-canyon multipath '
    'literature. '
    'A weighted-least-squares SPP correction experiment with three modes—baseline, '
    'NLOS exclusion, and ΔL correction—reveals that purely geometric ΔL correction '
    'does not improve GPS-only positioning under offline map conditions '
    '(baseline 75.6 m vs. correction 92.3 m mean horizontal error). '
    'Root-cause analysis attributes this outcome to GPS-only geometry collapse upon '
    'satellite exclusion, mixed-multipath amplitude uncertainty, and insufficient surface '
    'normal reliability (41.3% planarity < 0.5). '
    'These findings quantify the limitations of the geometric modeling layer and '
    'motivate signal-domain (SDR-based) validation as an essential complement, '
    'establishing a measurable baseline for future work.'
)

note('Keywords (5–7): GNSS NLOS, LiDAR point cloud, ray tracing, multipath, '
     'SPP pseudorange correction, urban canyon, UrbanNav')

doc.add_paragraph()

# ══════════════════════════════════════════════════════════════════════
# 1. Introduction
# ══════════════════════════════════════════════════════════════════════
heading('1. Introduction')

body(
    'Global Navigation Satellite System (GNSS) single-point positioning (SPP) '
    'in urban canyons remains a challenging problem. Tall buildings reflect and '
    'occlude satellite signals, causing non-line-of-sight (NLOS) reception and '
    'code multipath errors that can exceed tens to hundreds of metres [1]. '
    'In dense urban environments such as Hong Kong\'s Tsim Sha Tsui (TST) district, '
    'GPS-only SPP routinely exhibits horizontal errors of 50–200 m [2], '
    'rendering it insufficient for navigation applications that require sub-10 m accuracy.'
)

body(
    'LiDAR sensors mounted on autonomous vehicles provide dense 3-D point clouds '
    'of the surrounding environment. Several recent studies have exploited such '
    'point clouds to identify NLOS satellites through ray-casting [3,4] and '
    'to estimate pseudorange corrections from detected building geometry [5]. '
    'Wen et al. [5] demonstrated a ~4× improvement in SPP accuracy using a '
    'real-time sliding-window LiDAR map with GPS+BeiDou+GLONASS+Galileo observations. '
    'However, the conditions under which geometric ΔL corrections are beneficial—and '
    'when they fail—have not been systematically quantified for GPS-only, '
    'offline-map scenarios.'
)

body(
    'This paper makes the following contributions:'
)
for i, c in enumerate([
    'We implement and evaluate a complete, reproducible LiDAR–GNSS NLOS '
    'detection-to-correction pipeline on the public UrbanNav dataset, '
    'providing step-by-step quantitative results at each stage.',
    'We introduce a cross-validation between LiDAR geometric NLOS labels and '
    'del2AINLOS machine-learning labels [6], identifying per-elevation-band '
    'reliability profiles of the LiDAR detector.',
    'We conduct a three-mode SPP correction experiment (baseline / NLOS exclusion / '
    'ΔL correction) and provide a structured root-cause analysis of the observed '
    'correction failure, directly motivating SDR-based signal-domain validation '
    'as a necessary complement to geometric modeling.',
], 1):
    body(f'({i+1}) {c}', indent=0.8)

note('Consider adding "The remainder of this paper is organised as follows..." at the end of this paragraph.')

# ══════════════════════════════════════════════════════════════════════
# 2. Related Work
# ══════════════════════════════════════════════════════════════════════
heading('2. Related Work')

heading('2.1  GNSS NLOS Detection', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Early approaches to NLOS mitigation relied on 3-D building models derived from '
    'GIS databases. Groves and Adjrad [7] proposed a likelihood-based method using '
    'LOS/NLOS predictions from 3-D mapping combined with skewed pseudorange error '
    'distributions. Ray-tracing in urban environments was formalised by '
    'Siebler et al. [8], who established the coordinate-frame transformations '
    'required to compute incidence angles from surface normals in autonomous-driving '
    'scenarios. LiDAR-based approaches emerged with Wen et al. [3,4], replacing '
    'GIS models with real-sensor point clouds for improved spatial accuracy. '
    'Machine-learning alternatives, exemplified by del2AINLOS [6], exploit '
    'double-difference pseudorange residuals to classify NLOS without explicit '
    'geometric modelling.'
)

heading('2.2  Pseudorange Correction via Reflection Geometry', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Lau and Cross [9] derived the extra path length formula ΔL = 2·d·cos θᵢ '
    'from mirror-image geometry for GNSS carrier-phase multipath and validated it '
    'against GPS signals reflected from controlled surfaces. Wen et al. [5] '
    'operationalised this correction using real-time LiDAR point clouds and '
    'multi-constellation GNSS, achieving horizontal RMS reductions from '
    '~25.9 m to ~6.1 m in Hong Kong urban canyons. The 2022 follow-up [4] '
    'extended the framework to RTK positioning via factor-graph optimisation. '
    'A critical, yet unresolved, gap in the literature is the quantitative '
    'relationship between the geometric ΔL estimate and the actual DLL tracking '
    'error, which depends on the amplitude ratio of reflected to direct signal—'
    'a quantity accessible only through signal-domain measurement (e.g., SDR).'
)

heading('2.3  Distinction from Prior Work', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Unlike Wen et al. [5], who use a real-time multi-constellation setup, '
    'this paper deliberately evaluates the GPS-only, offline-map regime to '
    'bound the applicability of geometric correction. The negative correction '
    'result reported here constitutes a quantified failure-mode analysis absent '
    'from existing literature, and directly motivates SDR-based amplitude '
    'measurement as the missing layer.'
)

# ══════════════════════════════════════════════════════════════════════
# 3. Methodology
# ══════════════════════════════════════════════════════════════════════
heading('3. Methodology')

body(
    'The proposed pipeline processes LiDAR bags and GNSS RINEX files through '
    'five sequential stages, illustrated in Fig. 1. All coordinate computations '
    'use the WGS-84 ellipsoid; the local reference frame is East-North-Up (ENU) '
    'with origin at the first NovAtel INSPVAX pose.'
)

note('[Fig. 1] Pipeline block diagram: RINEX obs/nav → Step 3 (azel) '
     '| LiDAR bag → Step 2 (map) → Step 4 (ray cast) → Step 6 (ΔL) → Step 7 (SPP)')

heading('3.1  LiDAR Map Construction (Step 2)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Point clouds from a Velodyne VLP-16 LiDAR are accumulated along the '
    'NovAtel INSPVAX trajectory (RTK/INS, ~5 cm accuracy). Each frame is '
    'transformed to the global ENU frame using the extrinsic calibration '
    'T_LiDAR_SPAN = [0, 0, 0.14 m] (vertical offset). The accumulated map '
    'is downsampled to 0.2 m voxels using Open3D, yielding 8.53 M points. '
    'Ground points with ENU-Z < 1.0 m are excluded in downstream stages to '
    'suppress vehicle-body reflections.'
)

heading('3.2  Satellite Geometry Computation (Step 3)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'For each RINEX observation epoch, satellite ECEF positions are computed '
    'from broadcast ephemerides using Keplerian orbit propagation and '
    'satellite clock correction (IS-GPS-200). Elevation and azimuth angles '
    'are derived at the receiver position interpolated from the INSPVAX '
    'trajectory. A minimum elevation mask of 5° is applied. '
    'Leap-second handling follows the convention documented in the dataset: '
    'the RINEX epoch time offset (18 s) is compensated when matching '
    'against UTC-referenced trajectory data.'
)

heading('3.3  Ray-Casting NLOS Detection (Step 4)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'The LiDAR map is voxelised at 0.5 m resolution into a 3-D occupancy set '
    'using a 21-bit packed uint64 key for O(1) lookup. For each satellite, '
    'a ray is cast from the receiver position in the direction of the satellite '
    '(ENU unit vector computed from azimuth and elevation). The ray starts at '
    'd_start = 5.0 m from the receiver to skip vehicle-body clutter in the '
    'accumulated offline map. A voxel intersection within the maximum range '
    '(80 m) is classified as NLOS; otherwise LOS. The intersection distance '
    'd_hit is recorded for downstream geometry computation.'
)

heading('3.4  Reflection Surface Modeling (Step 6)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'For each NLOS satellite, the signal hit point in ENU coordinates is '
    'reconstructed as P_hit = P_rx + d_hit · LOS_hat. A local surface normal '
    'n̂ is estimated by PCA on the k ≤ 30 nearest LiDAR map points within '
    'radius r = 3.0 m [10,11]. The normal is oriented toward the receiver. '
    'Planarity is quantified as 1 − λ_min/λ_max; estimates with planarity '
    '< 0.5 are flagged as unreliable (non-planar reflectors). '
    'The incidence angle θᵢ = arccos(|LOS_hat · n̂|) and the extra path '
    'length ΔL = 2·d_hit·cos θᵢ are computed per [9]. When planarity < 0.5 '
    'or fewer than 6 neighbours are found, a simplified fallback formula '
    'ΔL = 2·d_hit·cos(elevation) is applied, assuming a vertical facade '
    '(upper-bound estimate). Multipath severity is classified as '
    'mild (ΔL < 2 m), strong (2–10 m), or severe (≥ 10 m) following [12].'
)

heading('3.5  WLS SPP Correction (Step 7)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Single-point positioning is solved by iterative weighted least-squares (WLS) '
    'with four unknowns: receiver ECEF position and clock bias. The linearised '
    'observation equation per satellite i is:'
)
body('    δρᵢ = eᵢᵀ δx + c δtᵣ', indent=1.0)
body(
    'where eᵢ is the unit vector from satellite to receiver, δx the position '
    'correction vector, and c δtᵣ the receiver clock correction [1]. '
    'Pseudoranges are corrected for satellite clock bias (ρ_corr = ρ + c·δt_sv, '
    'Kaplan & Hegarty 2006 p.183) and simplified tropospheric delay '
    '(T = 2.3/sin(el) m). Sagnac effect correction is applied per IS-GPS-200. '
    'Elevation-based weighting wᵢ = sin²(elᵢ) is used [13].'
)
body('Three modes are evaluated:')
for label, desc in [
    ('Baseline', 'All satellites, no correction.'),
    ('NLOS Exclusion', 'LiDAR-NLOS satellites set weight = 0.'),
    ('ΔL Correction',
     'NLOS pseudoranges corrected by subtracting ΔL; weight '
     '× planarity × 0.1 following [5].'),
]:
    body(f'  • {label}: {desc}', indent=0.5)
body(
    'Estimated positions are compared against ground-truth INSPVAX positions '
    'interpolated to each epoch (nearest-neighbour, ≤10 s tolerance). '
    'Horizontal (2-D) and 3-D errors in the ENU frame are reported.'
)

# ══════════════════════════════════════════════════════════════════════
# 4. Experimental Setup
# ══════════════════════════════════════════════════════════════════════
heading('4. Experimental Setup')

heading('4.1  Dataset', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Experiments use the publicly available UrbanNav Medium-Urban-1 dataset [2], '
    'collected on 17 May 2021 in the Tsim Sha Tsui district of Hong Kong '
    '(approximately 22.301°N, 114.179°E). The dataset provides:'
)
for item in [
    'GNSS raw observations: u-blox F9P, RINEX 3 format, GPS L1 C/A, 657 epochs at 1 Hz;',
    'LiDAR: Velodyne VLP-16, 10 Hz, ROS bag format;',
    'Ground truth: NovAtel SPAN (RTK/INS), 787 poses at 1 Hz, ~5 cm horizontal accuracy;',
    'Reference NLOS labels: del2AINLOS [6], computed from double-difference GNSS residuals.',
]:
    body('  • ' + item, indent=0.5)

add_table(
    ['Parameter', 'Value'],
    [
        ['Dataset', 'UrbanNav Medium-Urban-1'],
        ['Location', 'Tsim Sha Tsui, Hong Kong'],
        ['Date', '2021-05-17, 02:33–02:46 UTC'],
        ['GNSS receiver', 'u-blox F9P (GPS L1 C/A)'],
        ['LiDAR', 'Velodyne VLP-16'],
        ['Ground truth', 'NovAtel INSPVAX (RTK/INS, ~5 cm)'],
        ['RINEX epochs', '657'],
        ['GT poses', '787 @ 1 Hz'],
        ['Approx. position', '22.301°N, 114.179°E, ~3.5 m'],
    ],
    caption='Table 1. Dataset parameters.'
)

heading('4.2  Software and Implementation', level=2, size=11, bold=True, color=(40,80,140))
body(
    'The pipeline is implemented in Python 3 and ROS Noetic (Docker container). '
    'Key libraries: NumPy, SciPy (KD-tree, WLS), Open3D (voxel downsampling), '
    'and rinex_utils (RINEX parsing, satellite position, del2AINLOS package). '
    'All scripts are open-sourced at: https://github.com/phyllis-TANG/gnss_ws '
    '(branch: claude/review-gnss-spp-JmtgD).'
)

heading('4.3  Evaluation Metrics', level=2, size=11, bold=True, color=(40,80,140))
body(
    'NLOS detection: Precision, Recall, F1-score against del2AINLOS labels. '
    'SPP accuracy: mean horizontal error, RMS, 50th and 95th percentiles. '
    'Statistical robustness of ΔL estimates is assessed by comparing the full '
    'dataset against the planarity ≥ 0.5 subset.'
)

# ══════════════════════════════════════════════════════════════════════
# 5. Results and Discussion
# ══════════════════════════════════════════════════════════════════════
heading('5. Results and Discussion')

heading('5.1  NLOS Detection Performance (Step 4 vs del2AINLOS)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Ray-casting on the offline LiDAR map yields an overall NLOS rate of 66.5% '
    '(2753/4140 satellite-epoch pairs). Compared against del2AINLOS labels, the '
    'confusion matrix is shown in Table 2.'
)
add_table(
    ['Metric', 'Value', 'Interpretation'],
    [
        ['True Positives (TP)',  '1091', 'LiDAR NLOS ∩ del2 NLOS'],
        ['False Positives (FP)', '1312', 'LiDAR NLOS, del2 LOS (over-detection)'],
        ['False Negatives (FN)', '307',  'LiDAR LOS, del2 NLOS (under-detection)'],
        ['True Negatives (TN)',  '633',  'LiDAR LOS ∩ del2 LOS'],
        ['Precision',            '0.454', '—'],
        ['Recall',               '0.780', '—'],
        ['F1-score',             '0.574', '—'],
    ],
    caption='Table 2. LiDAR NLOS detection vs del2AINLOS labels (all elevations).'
)
body(
    'The high recall (0.780) indicates the LiDAR detector rarely misses true NLOS '
    'signals, but the low precision (0.454) reveals substantial over-detection. '
    'Per-elevation analysis (Fig. 3) shows that F1 degrades to 0.242 for '
    'satellites above 45°, where the offline accumulated map predicts NLOS '
    'due to distant structures not present in the real-time environment. '
    'This is consistent with the known limitation of offline maps '
    'noted in [4]: accumulated point clouds include building surfaces beyond '
    'the instantaneous LiDAR field of view, causing false NLOS predictions '
    'for high-elevation satellites.'
)
note('[Fig. 3] Precision/Recall/F1 bar chart by elevation bin + confusion matrix heatmap (from lidar_nlos_comparison.html)')

heading('5.2  Reflection Geometry and ΔL Distribution (Step 6)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Surface normal estimation succeeds for 99.9% of NLOS satellite-epoch pairs '
    '(2750/2753), but 41.3% of estimates have planarity < 0.5, flagging unreliable '
    'normals at building edges, vehicle rooftops, and vegetation. '
    'The estimated ΔL distribution (Table 3, Fig. 4) is dominated by the '
    '"strong" multipath category (2–10 m, 67.9%), consistent with urban-canyon '
    'multipath delay statistics reported by Steingass and Lehner [12] for Munich. '
    'Robustness is confirmed by restricting to planarity ≥ 0.5: the mean '
    'changes by only 0.03 m (6.70 → 6.67 m) and the severity fractions '
    'shift by < 0.3 percentage points, indicating that unreliable normal '
    'estimates introduce random rather than systematic error.'
)
add_table(
    ['Category', 'ΔL Range', 'Count', 'Fraction', 'Physical Implication'],
    [
        ['mild',   '< 2 m',    '402',  '14.6%', 'DLL-trackable, small SPP impact'],
        ['strong', '2–10 m',   '1868', '67.9%', 'Dominant urban multipath regime'],
        ['severe', '≥ 10 m',   '483',  '17.5%', 'Pseudorange unusable without correction'],
        ['All',    '0–38 m',   '2753', '100%',  'Mean 6.70 m, median 6.45 m'],
    ],
    caption='Table 3. Multipath severity distribution from geometric ΔL estimates.'
)
note('[Fig. 4] ΔL vs elevation scatter plot + severity donut chart (from lidar_reflection_model.html)')

heading('5.3  SPP Correction Experiment (Step 7)', level=2, size=11, bold=True, color=(40,80,140))
body(
    'Table 4 summarises SPP horizontal error across the three modes for 635 '
    'matched epochs (GT tolerance 10 s, elevation mask 10°).'
)
add_table(
    ['Mode', 'Valid epochs', 'Mean (m)', 'RMS (m)', '50th (m)', '95th (m)'],
    [
        ['Baseline (no correction)', '635', '75.6',  '204.3', '47.2',  '116.8'],
        ['NLOS Exclusion',           '85',  '238.9', '655.9', '95.5',  '1154.4'],
        ['ΔL Correction',            '635', '92.3',  '205.6', '68.7',  '141.2'],
    ],
    caption='Table 4. SPP horizontal error comparison across three modes.'
)
body(
    'The ΔL correction mode shows a slight degradation relative to baseline '
    '(+16.7 m mean, +1.3 m RMS), while NLOS exclusion produces severe degradation '
    'for the 85 epochs where sufficient satellites remain (mean +163 m, '
    'RMS +452 m). The CDF and time-series plots (Fig. 5–6) confirm these trends '
    'are consistent across the entire trajectory.'
)
note('[Fig. 5] Horizontal error CDF for three modes (from spp_correction_report.html)')
note('[Fig. 6] Horizontal error time series (from spp_correction_report.html)')

heading('5.4  Root-Cause Analysis', level=2, size=11, bold=True, color=(40,80,140))
body(
    'The correction failure can be attributed to four compounding factors:'
)
causes = [
    ('GPS-only geometry collapse',
     'With 9 visible GPS satellites and a 66.5% NLOS rate, exclusion leaves '
     '~3 satellites per epoch on average, below the WLS minimum of 4. Only '
     '85/635 epochs (13%) survive exclusion with adequate geometry. '
     'Wen et al. [5] avoided this by using multi-constellation GNSS '
     '(GPS+BDS+GLONASS+Galileo), retaining 8–12 LOS satellites after exclusion.'),
    ('Geometric ΔL ≠ DLL tracking error in mixed-multipath scenarios',
     'For pure NLOS (direct path fully blocked), the DLL error equals ΔL '
     'regardless of signal amplitude [14]. However, for partial NLOS '
     '(direct + reflected signal present), the error is '
     'δρ = ΔL · f(A_r/A_d), where A_r/A_d is the reflected-to-direct '
     'amplitude ratio—a quantity not available from geometry alone. '
     'Applying full ΔL correction to partially obstructed satellites '
     'overcorrects and introduces additional bias. The del2AINLOS '
     'Precision of 0.454 implies ~55% of LiDAR-NLOS labels are actually '
     'LOS or partial NLOS, making overcorrection the dominant error source.'),
    ('Offline map surface normal reliability',
     'The accumulated LiDAR map contains building facades, vegetation, '
     'vehicles, and road markings from the full trajectory. Local PCA at '
     'hit points intersecting multi-surface regions (edges, corners, '
     'accumulated clutter) yields planarity < 0.5 for 41.3% of estimates, '
     'meaning these ΔL values are geometrically ill-defined.'),
    ('NLOS weight reduction suppresses correction contribution',
     'The down-weighting scheme (w_NLOS = sin²(el) × planarity × 0.1) '
     'assigns ~5% of LOS weight to corrected NLOS satellites. Even when '
     'ΔL estimates are accurate, the corrected pseudoranges contribute '
     'minimally to the WLS solution, diluting the correction benefit.'),
]
for i, (title, text) in enumerate(causes, 1):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.left_indent  = Cm(0.5)
    r = p.add_run(f'  ({i}) {title}: ')
    set_font(r, size=11, bold=True)
    r2 = p.add_run(text)
    set_font(r2, size=11)

body(
    'In summary, the negative correction result is expected and reproducible '
    'under the GPS-only, offline-map experimental conditions. It does not '
    'invalidate the geometric modeling pipeline; rather, it delineates the '
    'boundary conditions beyond which geometric correction alone is insufficient '
    'and signal-domain characterisation becomes necessary.',
    space_after=8
)

# ══════════════════════════════════════════════════════════════════════
# 6. Conclusion and Future Work
# ══════════════════════════════════════════════════════════════════════
heading('6. Conclusion and Future Work')

body(
    'This paper presented a complete, open-source LiDAR–GNSS NLOS pipeline '
    'evaluated on the UrbanNav Medium-Urban-1 dataset. The geometric detector '
    'achieves F1 = 0.574 against del2AINLOS ML labels and estimates multipath '
    'extra path lengths with a mean of 6.70 m, predominantly in the '
    '"strong" category (2–10 m, 68%). A three-mode SPP correction experiment '
    'demonstrates that purely geometric ΔL correction does not improve '
    'GPS-only positioning under offline-map conditions, and the root-cause '
    'analysis identifies GPS geometry collapse, mixed-multipath amplitude '
    'ambiguity, and offline map normal reliability as the key limiting factors.'
)
body(
    'Future work will address these limitations along three directions:'
)
for item in [
    'SDR-based signal-domain validation: a planned open-sky experiment using '
    'a USRP N210 SDR will capture raw GNSS IF data and reconstruct correlation '
    'functions via dense-tap DLL (61 taps, 0.05-chip spacing). Comparing '
    'measured extra delay ΔL_SDR = Δτ·c against geometric ΔL_geom will '
    'directly quantify the geometry-to-signal accuracy gap and enable '
    'amplitude-weighted pseudorange correction.',
    'Real-time sliding-window LiDAR map: replacing the offline accumulated map '
    'with a 250 m sliding window following [4] is expected to eliminate '
    'false NLOS predictions for high-elevation satellites and improve '
    'surface normal reliability.',
    'Multi-constellation extension: incorporating BeiDou and Galileo '
    'observations (requiring corresponding CORS navigation files) will '
    'increase the available satellite count from ~9 to ~20 per epoch, '
    'making NLOS exclusion geometrically viable.',
]:
    body('  • ' + item, indent=0.5)

# ══════════════════════════════════════════════════════════════════════
# References
# ══════════════════════════════════════════════════════════════════════
heading('References')
refs = [
    '[1] Kaplan, E. & Hegarty, C. (2006). Understanding GPS: Principles and Applications, 2nd ed. Artech House.',
    '[2] Hsu, L.-T. et al. (2021). UrbanNav: An open-source multisensory dataset for benchmarking positioning algorithms designed for urban areas. ION GNSS+ 2021.',
    '[3] Wen, W. et al. (2018). 3D LiDAR aided GNSS NLOS exclusion. ION GNSS+ 2018.',
    '[4] Wen, W. et al. (2022). 3D LiDAR aided GNSS NLOS mitigation for reliable GNSS-RTK positioning in urban canyons. arXiv:2212.05477.',
    '[5] Wen, W. et al. (2019). Correcting NLOS by 3D LiDAR and building height to improve GNSS single point positioning. NAVIGATION, 66(4). doi:10.1002/navi.335.',
    '[6] del2AINLOS: Double-Difference AI-based NLOS Classification. PSRI-73-2309-PR-Dev package, rospak/src/del2AINLOS.',
    '[7] Groves, P.D. & Adjrad, M. (2017). Likelihood-based GNSS positioning using LOS/NLOS predictions from 3D mapping. GPS Solutions, 21(4). doi:10.1007/s10291-017-0654-1.',
    '[8] Siebler, B. et al. (2023). Coordinate frames and transformations in GNSS ray-tracing for autonomous driving. Remote Sensing, 15(1). doi:10.3390/rs15010180.',
    '[9] Lau, L. & Cross, P. (2007). Development and testing of a new ray-tracing approach to GNSS carrier-phase multipath modelling. Journal of Geodesy, 81(11). doi:10.1007/s00190-007-0139-z.',
    '[10] Wen, W. et al. (2021). 3D LiDAR aided GNSS and its tightly coupled integration with INS via factor graph optimisation. ICRA. arXiv:2106.01594.',
    '[11] Hoppe, H. et al. (1992). Surface reconstruction from unorganized points. SIGGRAPH 1992.',
    '[12] Steingass, A. & Lehner, A. (2004). Measuring the navigation multipath channel — a statistical analysis. ION GNSS 2004.',
    '[13] Groves, P.D. (2013). Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, 2nd ed. Artech House.',
    '[14] Braasch, M.S. (1996). Multipath effects. In: Parkinson & Spilker (eds.), Global Positioning System: Theory and Applications, Vol. 1.',
]
for ref in refs:
    p = doc.add_paragraph()
    p.paragraph_format.left_indent    = Cm(1.0)
    p.paragraph_format.first_line_indent = Cm(-1.0)
    p.paragraph_format.space_after   = Pt(3)
    run = p.add_run(ref)
    set_font(run, size=9.5)

# ══════════════════════════════════════════════════════════════════════
# Appendix: Figure and Table Checklist
# ══════════════════════════════════════════════════════════════════════
doc.add_page_break()
heading('Appendix: Figure and Table Checklist', size=12)
note('All figures below are generated by pipeline scripts; screenshots or vector exports can be inserted directly into the paper.')
add_table(
    ['ID', 'Content', 'Source script / file', 'Status'],
    [
        ['Fig. 1', 'Pipeline architecture block diagram', 'Hand-drawn / PowerPoint',         'To be created'],
        ['Fig. 2', 'CloudCompare LiDAR map screenshot',   'urbannav_map.ply',                'Screenshot available'],
        ['Fig. 3', 'NLOS detection performance by elevation bin', 'lidar_nlos_comparison.html',    'Generated'],
        ['Fig. 4', 'ΔL distribution + severity chart',   'lidar_reflection_model.html',      'Generated'],
        ['Fig. 5', 'SPP three-mode horizontal error CDF', 'spp_correction_report.html',       'Generated'],
        ['Fig. 6', 'SPP horizontal error time series',    'spp_correction_report.html',       'Generated'],
        ['Table 1','Dataset parameters',                  'This document',                    'Complete'],
        ['Table 2','NLOS detection confusion matrix',     'This document',                    'Complete'],
        ['Table 3','ΔL severity distribution',            'This document',                    'Complete'],
        ['Table 4','SPP error comparison',                'This document',                    'Complete'],
    ],
    caption='Appendix Table. Figure and table checklist with completion status.'
)

doc.save(args.out)
print(f'Saved: {args.out}')
