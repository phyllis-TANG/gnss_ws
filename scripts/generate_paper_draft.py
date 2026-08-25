#!/usr/bin/env python3
"""
generate_paper_draft.py
Generate paper draft Word document (python-docx)
Run: python3 generate_paper_draft.py --out paper_draft.docx
"""
import argparse
from docx import Document
from docx.shared import Pt, Cm, RGBColor
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
section.page_width    = Cm(21.0)
section.page_height   = Cm(29.7)
section.left_margin   = Cm(2.5)
section.right_margin  = Cm(2.5)
section.top_margin    = Cm(2.5)
section.bottom_margin = Cm(2.5)

# ── Style helpers ────────────────────────────────────────────────────
def set_font(run, name='Times New Roman', size=11, bold=False, italic=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run.bold   = bold
    run.italic = italic
    if color:
        run.font.color.rgb = RGBColor(*color)

def heading(text, level=1, size=13, bold=True, color=None):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(14)
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
    p.paragraph_format.left_indent = Cm(1.0)
    p.paragraph_format.space_after = Pt(4)
    run = p.add_run('[DRAFT NOTE] ' + text)
    set_font(run, size=9.5, italic=True, color=(120, 120, 120))
    return p

def code_block(lines):
    """Monospaced code block with light-grey background."""
    for line in lines:
        p = doc.add_paragraph()
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.space_after  = Pt(0)
        p.paragraph_format.left_indent  = Cm(1.0)
        run = p.add_run(line if line else ' ')
        run.font.name = 'Courier New'
        run.font.size = Pt(9)
        # light grey background
        tc = p._p
        pPr = tc.get_or_add_pPr()
        shd = OxmlElement('w:shd')
        shd.set(qn('w:fill'), 'F2F2F2')
        shd.set(qn('w:val'),  'clear')
        pPr.append(shd)
    doc.add_paragraph().paragraph_format.space_after = Pt(4)

def add_table(headers, rows, caption=''):
    if caption:
        cp = doc.add_paragraph()
        cp.paragraph_format.space_before = Pt(8)
        r = cp.add_run(caption)
        set_font(r, size=10, bold=True)
    tbl = doc.add_table(rows=1 + len(rows), cols=len(headers))
    tbl.style     = 'Table Grid'
    tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    # Header row
    for i, h in enumerate(headers):
        cell = tbl.rows[0].cells[i]
        cell.paragraphs[0].clear()
        run = cell.paragraphs[0].add_run(h)
        set_font(run, size=10, bold=True)
        cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
        # Header background colour
        tc   = cell._tc
        tcPr = tc.get_or_add_tcPr()
        shd  = OxmlElement('w:shd')
        shd.set(qn('w:fill'), 'D6E4F0')
        shd.set(qn('w:val'),  'clear')
        tcPr.append(shd)
    # Data rows
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = tbl.rows[ri + 1].cells[ci]
            cell.paragraphs[0].clear()
            run = cell.paragraphs[0].add_run(str(val))
            set_font(run, size=10)
            cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    doc.add_paragraph()
    return tbl

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
    'LiDAR-Aided GNSS NLOS Geometric Modeling and\n'
    'Pseudorange Correction Evaluation in Urban Canyons  (Draft v0.2)')
set_font(r2, size=12, italic=True, color=(80, 80, 80))

p_author = doc.add_paragraph()
p_author.alignment = WD_ALIGN_PARAGRAPH.CENTER
r3 = p_author.add_run('[Author Names] · [Affiliation] · ' +
                      datetime.datetime.now().strftime('%Y-%m-%d'))
set_font(r3, size=10, color=(100, 100, 100))
doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════
# Abstract
# ══════════════════════════════════════════════════════════════════════
heading('Abstract', size=13)
body(
    'Urban canyon environments severely degrade GNSS positioning accuracy due to '
    'non-line-of-sight (NLOS) signal reception and multipath propagation. This paper '
    'presents an end-to-end, open-source pipeline that leverages LiDAR point cloud maps '
    'for GNSS NLOS detection, reflection surface modeling, and pseudorange correction, '
    'evaluated on the publicly available UrbanNav Medium-Urban-1 dataset collected in '
    'Tsim Sha Tsui, Hong Kong. '
    'The pipeline comprises five stages: (1) INSPVAX trajectory extraction, '
    '(2) dense LiDAR map construction (8.53 M points, 0.2 m voxel), '
    '(3) satellite azimuth/elevation computation from RINEX observations, '
    '(4) voxel-grid ray-casting NLOS detection, and '
    '(5) PCA-based reflection surface modeling yielding per-satellite extra path '
    'length estimates (ΔL = 2·d·cos θᵢ). '
    'Compared against del2AINLOS machine-learning labels, the LiDAR detector achieves '
    'Precision = 0.454, Recall = 0.780, F1 = 0.574. '
    'The geometric ΔL distribution is dominated by the "strong" multipath category '
    '(2–10 m, 67.9%) with a mean of 6.70 m. '
    'A three-mode WLS SPP experiment (baseline / NLOS exclusion / ΔL correction) '
    'reveals that GPS-only geometric correction does not improve positioning accuracy '
    '(baseline 75.6 m vs. correction 92.3 m mean horizontal error). '
    'Root-cause analysis identifies GPS geometry collapse upon satellite exclusion, '
    'mixed-multipath amplitude uncertainty, and 41.3% low-planarity surface normals '
    'as the key failure mechanisms. '
    'These findings motivate a companion multi-constellation experiment (GPS + BeiDou '
    '+ Galileo) that is expected to close the geometry gap and validate the pipeline '
    'under operationally relevant conditions.'
)
note('Keywords (5–7): GNSS NLOS, LiDAR point cloud, ray tracing, multipath, '
     'WLS SPP, urban canyon, UrbanNav')
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
    'rendering it insufficient for navigation applications requiring sub-10 m accuracy.'
)
body(
    'LiDAR sensors mounted on autonomous vehicles provide dense 3-D point clouds '
    'of the surrounding environment. Recent studies have exploited such point clouds '
    'to identify NLOS satellites through ray-casting [3,4] and to estimate pseudorange '
    'corrections from detected building geometry [5]. '
    'Wen et al. [5] demonstrated a ~4× improvement in SPP accuracy using a '
    'real-time sliding-window LiDAR map with multi-constellation GNSS. '
    'However, the failure modes of geometric ΔL correction under GPS-only, '
    'offline-map conditions have not been systematically quantified.'
)
body('This paper makes the following contributions:')
for i, c in enumerate([
    'A complete, reproducible LiDAR–GNSS NLOS pipeline on the public UrbanNav '
    'dataset, with step-by-step quantitative results and all source code released.',
    'Cross-validation between LiDAR geometric NLOS labels and del2AINLOS ML labels [6], '
    'identifying per-elevation-band reliability profiles.',
    'A three-mode SPP correction experiment with structured root-cause analysis of '
    'the GPS-only correction failure, directly motivating multi-constellation '
    'follow-up experiments.',
], 1):
    body(f'({i+1}) {c}', indent=0.8)

note('Add "The remainder of this paper is organised as follows..." at paragraph end.')

# ══════════════════════════════════════════════════════════════════════
# 2. Related Work
# ══════════════════════════════════════════════════════════════════════
heading('2. Related Work')

heading('2.1  GNSS NLOS Detection', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Early NLOS mitigation relied on GIS-derived 3-D building models. Groves and '
    'Adjrad [7] proposed likelihood-based positioning using skewed pseudorange error '
    'distributions conditioned on LOS/NLOS predictions. Siebler et al. [8] formalised '
    'coordinate-frame transformations for GNSS ray-tracing in autonomous-driving '
    'scenarios. LiDAR-based approaches emerged with Wen et al. [3,4], replacing GIS '
    'models with real-sensor point clouds for improved spatial accuracy. '
    'Machine-learning alternatives, exemplified by del2AINLOS [6], classify NLOS '
    'from double-difference pseudorange residuals without explicit geometric modelling.'
)

heading('2.2  Pseudorange Correction via Reflection Geometry', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Lau and Cross [9] derived the extra path length formula ΔL = 2·d·cos θᵢ '
    'from mirror-image geometry and validated it against GPS signals reflected from '
    'controlled surfaces. Wen et al. [5] operationalised this with real-time LiDAR '
    'and multi-constellation GNSS, achieving horizontal RMS reduction from ~25.9 m '
    'to ~6.1 m in Hong Kong. A critical unresolved gap is the quantitative relationship '
    'between geometric ΔL and actual DLL tracking error, which depends on the '
    'amplitude ratio of reflected to direct signal — accessible only through '
    'signal-domain measurement (e.g., SDR).'
)

heading('2.3  Distinction from Prior Work', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Unlike Wen et al. [5] who use a real-time multi-constellation setup, '
    'this paper evaluates the GPS-only, offline-map regime to bound the applicability '
    'of geometric correction. The negative correction result constitutes a quantified '
    'failure-mode analysis absent from existing literature. The companion '
    'multi-constellation experiment (Section 5.5) will directly validate whether '
    'the pipeline produces positive results once the geometry constraint is lifted.'
)

# ══════════════════════════════════════════════════════════════════════
# 3. Methodology
# ══════════════════════════════════════════════════════════════════════
heading('3. Methodology')
body(
    'The proposed pipeline processes LiDAR bags and GNSS RINEX files through '
    'five sequential stages (Fig. 1). All coordinates use WGS-84; the local frame '
    'is East-North-Up (ENU) with origin at the first NovAtel INSPVAX pose.'
)
note('[Fig. 1] Pipeline block diagram: RINEX obs/nav → Step 3 (azel) | '
     'LiDAR bag → Step 2 (map) → Step 4 (ray cast) → Step 6 (ΔL) → Step 7 (SPP)')

# ── 3.1 ──────────────────────────────────────────────────────────────
heading('3.1  LiDAR Map Construction (Step 2)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Point clouds from a Velodyne VLP-16 LiDAR are accumulated along the NovAtel '
    'INSPVAX trajectory (RTK/INS, ~5 cm accuracy). Each frame is transformed to '
    'the global ENU frame using T_LiDAR_SPAN = [0, 0, 0.14 m] (vertical offset). '
    'The map is downsampled to 0.2 m voxels via Open3D, yielding 8.53 M points. '
    'Ground points with ENU-Z < 1.0 m are excluded to suppress vehicle-body reflections.'
)
note('[Fig. 2] CloudCompare screenshot of urbannav_map.pcd (colour by height); '
     '8.53 M points shown in ENU frame, Z-range approx. −2 m to +80 m.')

# ── 3.2 ──────────────────────────────────────────────────────────────
heading('3.2  Satellite Geometry Computation (Step 3)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'For each RINEX epoch, satellite ECEF positions are computed from broadcast '
    'ephemerides using Keplerian orbit propagation. Satellite clock bias is applied '
    'following the sign convention of Kaplan & Hegarty [1] (Equation 1). '
    'The Sagnac effect is corrected by rotating the satellite ECEF vector about the '
    'Z-axis by Ω_E × travel_time per IS-GPS-200 (Equation 2). Elevation and azimuth '
    'are derived at the receiver position interpolated from the INSPVAX trajectory. '
    'A 5° elevation mask is applied; leap-second offset (18 s) is compensated when '
    'matching RINEX GPS time against UTC-referenced trajectory data.'
)
body('Key corrections (Python, lidar_step3_compute_azel.py):', size=10, italic=True)
code_block([
    '# Eq.1 — satellite clock correction (Kaplan & Hegarty 2006, p.183)',
    '# rho = r + c*(dt_rx - dt_sv)  =>  rho_corr = rho + c*dt_sv',
    'psr_clk = psr_raw + C_LIGHT * dt_sv',
    '',
    '# Eq.2 — Sagnac effect correction (IS-GPS-200)',
    'def sagnac_correct(sat_ecef, travel_time_s):',
    '    theta = OMEGA_E * travel_time_s          # Earth rotation angle',
    '    c, s  = math.cos(theta), math.sin(theta)',
    '    R = np.array([[c,  s, 0],',
    '                  [-s, c, 0],',
    '                  [0,  0, 1]])',
    '    return R @ sat_ecef',
])

# ── 3.3 ──────────────────────────────────────────────────────────────
heading('3.3  Ray-Casting NLOS Detection (Step 4)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'The LiDAR map is voxelised at 0.5 m resolution into a 3-D occupancy hash set '
    'using a 21-bit packed uint64 key for O(1) lookup. For each satellite, a ray is '
    'cast from the receiver ENU position toward the satellite. The ray starts at '
    'd_start = 5.0 m to skip vehicle-body clutter; a voxel hit within 80 m is '
    'classified NLOS and the intersection distance d_hit is recorded.'
)
body('Core implementation (Python, lidar_step4_ray_casting.py):', size=10, italic=True)
code_block([
    '# Pack (xi, yi, zi) into a single uint64 for O(1) set lookup',
    'def voxel_key(ex, ey, ez, res=0.5):',
    '    xi = int(math.floor(ex / res))',
    '    yi = int(math.floor(ey / res))',
    '    zi = int(math.floor(ez / res))',
    '    return ((xi & 0x1FFFFF) << 42) | ((yi & 0x1FFFFF) << 21) | (zi & 0x1FFFFF)',
    '',
    'def ray_cast(origin_enu, los_hat, voxel_set,',
    '             d_start=5.0, d_max=80.0, step=0.4):',
    '    t = d_start',
    '    while t <= d_max:',
    '        pt = origin_enu + t * los_hat',
    '        if voxel_key(*pt) in voxel_set:',
    '            return True, t      # NLOS — return hit distance',
    '        t += step',
    '    return False, None          # LOS',
])

# ── 3.4 ──────────────────────────────────────────────────────────────
heading('3.4  Reflection Surface Modeling (Step 6)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'For each NLOS satellite the signal hit point is reconstructed as '
    'P_hit = P_rx + d_hit · LOS_hat. A local surface normal n̂ is estimated '
    'by PCA (SVD) on the k ≤ 30 nearest LiDAR map points within radius 3.0 m '
    '[9, 10]. Planarity is defined as 1 − λ_min / λ_max; estimates with '
    'planarity < 0.5 are flagged as unreliable. The extra path length is '
    'computed as ΔL = 2·d_hit·cos θᵢ where θᵢ = arccos(|LOS_hat · n̂|) [9]. '
    'A vertical-facade fallback (ΔL = 2·d_hit·cos(elevation)) is applied when '
    'planarity < 0.5 or fewer than 6 neighbours are found. '
    'Multipath severity follows Steingass & Lehner [12]: '
    'mild (ΔL < 2 m), strong (2–10 m), severe (≥ 10 m).'
)
body('Core implementation (Python, lidar_step6_reflection_model.py):', size=10, italic=True)
code_block([
    'def estimate_normal_pca(points):',
    '    """Return (unit_normal, planarity) via SVD on centred point set."""',
    '    centered  = points - points.mean(axis=0)',
    '    _, sv, Vt = np.linalg.svd(centered, full_matrices=False)',
    '    normal    = Vt[-1]                        # smallest singular vector',
    '    planarity = 1.0 - sv[-1] / (sv[0] + 1e-9)',
    '    return normal, float(planarity)',
    '',
    '# For each NLOS satellite:',
    'p_hit  = rx_enu + d_hit * los_hat             # hit point in ENU',
    'idx    = map_tree.query_ball_point(p_hit, r=3.0)',
    'normal, planarity = estimate_normal_pca(map_pts[idx])',
    'if np.dot(normal, rx_enu - p_hit) < 0:        # orient toward receiver',
    '    normal = -normal',
    'if planarity >= 0.5 and len(idx) >= 6:',
    '    cos_theta = abs(np.dot(los_hat, normal))',
    '    delta_l   = 2.0 * d_hit * cos_theta       # Lau & Cross 2007',
    'else:',
    '    delta_l   = 2.0 * d_hit * math.cos(math.radians(elevation))  # fallback',
])

# ── 3.5 ──────────────────────────────────────────────────────────────
heading('3.5  WLS SPP Correction (Step 7)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Single-point positioning is solved by iterative WLS with four unknowns: '
    'receiver ECEF position and clock bias. The linearised observation equation '
    'per satellite i is:  δρᵢ = eᵢᵀ δx + c δtᵣ,  where eᵢ is the unit vector '
    'from satellite to receiver and c δtᵣ the receiver clock correction [1]. '
    'Pseudoranges are corrected for satellite clock bias and simplified tropospheric '
    'delay T = 2.3/sin(el) m. Elevation-based weighting wᵢ = sin²(elᵢ) is used [13].'
)
body('Three correction modes are evaluated:', size=11)
for label, desc in [
    ('Baseline',       'All satellites, elevation-based weights only.'),
    ('NLOS Exclusion', 'LiDAR-NLOS satellites set to weight = 0 (excluded).'),
    ('ΔL Correction',  'NLOS pseudoranges reduced by ΔL; '
                       'weight × planarity × 0.1 following [5].'),
]:
    body(f'  • {label}: {desc}', indent=0.5)
body('WLS solver (Python, lidar_step7_spp_correction.py):', size=10, italic=True)
code_block([
    'def wls_spp(sats_info, x0_ecef, max_iter=10):',
    '    x = np.array(list(x0_ecef) + [0.0])   # [X, Y, Z, c*dt_r]',
    '    for _ in range(max_iter):',
    '        H, dp, W = [], [], []',
    '        for s in sats_info:',
    '            diff = x[:3] - np.array(s["sat_ecef"])',
    '            r    = np.linalg.norm(diff)',
    '            if r < 1e4: continue           # skip invalid geometry',
    '            e    = diff / r                # unit line-of-sight vector',
    '            H.append([e[0], e[1], e[2], 1.0])',
    '            dp.append(s["psr_corr"] - r - x[3])',
    '            W.append(s["weight"])',
    '        H, dp = np.array(H), np.array(dp)',
    '        HtW   = H.T @ np.diag(W)',
    '        delta = np.linalg.solve(HtW @ H, HtW @ dp)',
    '        x    += delta',
    '        if np.linalg.norm(delta[:3]) < 0.01: break   # 1 cm convergence',
    '    return x[:3], x[3]   # ECEF position, clock bias',
])
body(
    'Ground-truth comparison: estimated ECEF positions are converted to ENU, '
    'then horizontal (2-D) and 3-D errors are computed against INSPVAX poses '
    'matched by nearest-neighbour timestamp (tolerance ≤ 10 s).'
)

# ══════════════════════════════════════════════════════════════════════
# 4. Experimental Setup
# ══════════════════════════════════════════════════════════════════════
heading('4. Experimental Setup')

heading('4.1  Dataset', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Experiments use the publicly available UrbanNav Medium-Urban-1 dataset [2], '
    'collected on 17 May 2021 in the Tsim Sha Tsui district of Hong Kong '
    '(approx. 22.301°N, 114.179°E). The dataset provides:'
)
for item in [
    'GNSS raw observations: u-blox F9P (multi-constellation capable), RINEX 3 format, '
    'GPS L1 C/A, 657 epochs at 1 Hz;',
    'LiDAR: Velodyne VLP-16, 10 Hz, ROS bag format;',
    'Ground truth: NovAtel SPAN (RTK/INS), 787 poses at 1 Hz, ~5 cm horizontal accuracy;',
    'Reference NLOS labels: del2AINLOS [6], from double-difference GNSS residuals.',
]:
    body('  • ' + item, indent=0.5)

add_table(
    ['Parameter', 'Value'],
    [
        ['Dataset',         'UrbanNav Medium-Urban-1'],
        ['Location',        'Tsim Sha Tsui, Hong Kong'],
        ['Date / time',     '2021-05-17, 02:33–02:46 UTC'],
        ['GNSS receiver',   'u-blox F9P (GPS L1 C/A used in Exp. A)'],
        ['LiDAR',           'Velodyne VLP-16'],
        ['Ground truth',    'NovAtel INSPVAX (RTK/INS, ~5 cm)'],
        ['RINEX epochs',    '657'],
        ['GT poses',        '787 @ 1 Hz'],
        ['LiDAR map',       '8.53 M pts, 0.2 m voxel, ENU frame'],
        ['Approx. position','22.301°N, 114.179°E, ~3.5 m altitude'],
    ],
    caption='Table 1. Dataset parameters.'
)

heading('4.2  Software and Implementation', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'The pipeline is implemented in Python 3 with ROS Noetic (Docker: ros1_gnss). '
    'Libraries: NumPy, SciPy (cKDTree, linalg), Open3D (voxel downsampling), '
    'rinex_utils / gnss_comm (RINEX parsing, satellite positions, del2AINLOS). '
    'Source code: https://github.com/phyllis-TANG/gnss_ws '
    '(branch: claude/review-gnss-spp-JmtgD). '
    'All pipeline scripts reside in gnss_ws/scripts/lidar_stepN_*.py.'
)

heading('4.3  Experiment Design', level=2, size=11, bold=True, color=(40, 80, 140))
body('Two experiments are defined:')
for label, desc in [
    ('Experiment A — GPS-only (completed)',
     'Navigation file: hksc137c.21n (GPS). ~9 satellites per epoch. '
     'Three SPP modes: baseline / NLOS exclusion / ΔL correction.'),
    ('Experiment B — Multi-constellation (planned)',
     'Navigation file: RINEX 3 mixed nav (GPS + BeiDou + Galileo, same CORS station). '
     '~18–22 satellites per epoch. Same three SPP modes, extended clock-bias vector.'),
]:
    p = doc.add_paragraph()
    p.paragraph_format.left_indent  = Cm(0.5)
    p.paragraph_format.space_before = Pt(3)
    r  = p.add_run(f'  • {label}: ')
    set_font(r, size=11, bold=True)
    r2 = p.add_run(desc)
    set_font(r2, size=11)
note('To obtain the RINEX 3 mixed nav file for 2021-05-17 DOY 137: '
     'download hksc137c.21p (or hksc1370.21p) from the Hong Kong Geodetic Survey '
     'CORS portal (https://www.geodetic.gov.hk) or IGS MGEX data centre. '
     'The u-blox F9P obs file already contains BeiDou / Galileo observations.')

heading('4.4  Evaluation Metrics', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'NLOS detection: Precision, Recall, F1-score vs del2AINLOS labels. '
    'SPP accuracy: mean horizontal error, RMS, 50th and 95th percentiles (all in metres). '
    'ΔL robustness: mean and severity fractions for full dataset vs planarity ≥ 0.5 subset.'
)

# ══════════════════════════════════════════════════════════════════════
# 5. Results and Discussion
# ══════════════════════════════════════════════════════════════════════
heading('5. Results and Discussion')

# ── 5.1 ──────────────────────────────────────────────────────────────
heading('5.1  NLOS Detection Performance (Step 4 vs del2AINLOS)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Ray-casting on the offline LiDAR map yields 2753 NLOS satellite-epoch pairs '
    'out of 4140 total (66.5% NLOS rate). Compared against del2AINLOS labels:'
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
    'High recall (0.780) confirms the LiDAR detector rarely misses true NLOS signals. '
    'Low precision (0.454) reflects over-detection: the offline accumulated map '
    'includes distant building surfaces not present in the instantaneous real-time '
    'environment, causing false NLOS for high-elevation satellites (el > 45°, '
    'F1 = 0.242). This is the known offline-map limitation documented in [4].'
)
note('[Fig. 3] Precision/Recall/F1 bar chart by elevation bin (0–15°, 15–30°, '
     '30–45°, >45°) + overall confusion matrix heatmap. Source: lidar_nlos_comparison.html')

# ── 5.2 ──────────────────────────────────────────────────────────────
heading('5.2  Reflection Geometry and ΔL Distribution (Step 6)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Surface normal estimation succeeds for 2750/2753 NLOS pairs (99.9%). '
    'However, 41.3% of estimates have planarity < 0.5, flagging unreliable normals '
    'at building edges, vegetation, and accumulated vehicle clutter in the offline map.'
)
add_table(
    ['Category', 'ΔL Range', 'Count', 'Fraction', 'Physical Implication'],
    [
        ['mild',   '< 2 m',  '402',  '14.6%', 'DLL-trackable; small SPP impact'],
        ['strong', '2–10 m', '1868', '67.9%', 'Dominant urban multipath regime'],
        ['severe', '≥ 10 m', '483',  '17.5%', 'Pseudorange degraded without correction'],
        ['All',    '0–38 m', '2753', '100%',  'Mean 6.70 m, median 6.45 m'],
    ],
    caption='Table 3. Multipath severity distribution from geometric ΔL estimates (Step 6).'
)
body(
    'Robustness check: restricting to planarity ≥ 0.5 (1617 pairs, 58.7%), '
    'the mean ΔL changes by only 0.03 m (6.70 → 6.67 m) and severity fractions '
    'shift by < 0.3 pp, confirming that low-planarity estimates introduce random '
    'rather than systematic error into the ΔL statistics.'
)
note('[Fig. 4] ΔL vs elevation scatter (colour by severity) + severity donut chart. '
     'Source: lidar_reflection_model.html')

# ── 5.3 ──────────────────────────────────────────────────────────────
heading('5.3  Experiment A — GPS-Only SPP Correction (Step 7)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Table 4 summarises horizontal SPP error across three modes for 635 matched epochs '
    '(GT tolerance 10 s, elevation mask 10°).'
)
add_table(
    ['Mode', 'Valid epochs', 'Mean (m)', 'RMS (m)', '50th (m)', '95th (m)'],
    [
        ['Baseline (no correction)',  '635', '75.6',  '204.3', '47.2',  '116.8'],
        ['NLOS Exclusion',            '85',  '238.9', '655.9', '95.5',  '1154.4'],
        ['ΔL Correction',             '635', '92.3',  '205.6', '68.7',  '141.2'],
    ],
    caption='Table 4. Experiment A — SPP horizontal error, GPS-only, three modes.'
)
body(
    'ΔL correction shows slight degradation (+16.7 m mean, +1.3 m RMS vs baseline). '
    'NLOS exclusion produces severe degradation for the 85 epochs that survive '
    '(mean +163 m, RMS +452 m). Both results are explained in Section 5.4.'
)
note('[Fig. 5] CDF of horizontal error for three modes. '
     '[Fig. 6] Horizontal error time series (635 epochs). '
     'Source: spp_correction_report.html')

# ── 5.4 ──────────────────────────────────────────────────────────────
heading('5.4  Root-Cause Analysis', level=2, size=11, bold=True, color=(40, 80, 140))
body('Four compounding factors explain the GPS-only correction failure:')
causes = [
    ('GPS-only geometry collapse',
     'With ~9 GPS satellites and a 66.5% NLOS rate, exclusion leaves ~3 LOS '
     'satellites per epoch — below the WLS minimum of 4. Only 85/635 epochs (13%) '
     'survive with adequate geometry. Wen et al. [5] avoided this by using '
     'GPS+BDS+GLONASS+Galileo, retaining 8–12 LOS satellites after exclusion. '
     'This is the single dominant failure cause.'),
    ('Geometric ΔL ≠ DLL tracking error in mixed multipath',
     'For pure NLOS (direct path fully blocked), the DLL error equals ΔL regardless '
     'of amplitude [14]. For partial NLOS (direct + reflected co-present), the error '
     'is δρ = ΔL · f(A_r/A_d), where A_r/A_d is the reflected-to-direct amplitude '
     'ratio — not available from geometry. del2AINLOS Precision = 0.454 implies '
     '~55% of LiDAR-NLOS labels are actually LOS or partial NLOS, so full ΔL '
     'correction overcorrects and introduces additional bias.'),
    ('Offline map surface normal reliability',
     'The accumulated LiDAR map contains vegetation, parked vehicles, and road '
     'markings from the full trajectory. Local PCA at multi-surface hit points '
     '(edges, corners) yields planarity < 0.5 for 41.3% of estimates, making '
     'those ΔL values geometrically ill-defined.'),
    ('NLOS weight suppression dilutes correction',
     'The weighting scheme w_NLOS = sin²(el) × planarity × 0.1 assigns ~5% of '
     'LOS weight to corrected NLOS satellites. Even when ΔL is accurate, the '
     'corrected pseudoranges contribute minimally to the WLS solution.'),
]
for i, (title, text) in enumerate(causes, 1):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.left_indent  = Cm(0.5)
    r  = p.add_run(f'  ({i}) {title}: ')
    set_font(r, size=11, bold=True)
    r2 = p.add_run(text)
    set_font(r2, size=11)
body(
    'In summary, the negative SPP result is expected and reproducible under '
    'GPS-only, offline-map conditions. It does not invalidate the pipeline; '
    'it delineates the boundary conditions under which geometric correction alone '
    'is insufficient, directly motivating Experiment B.',
    space_after=8
)

# ── 5.5 ──────────────────────────────────────────────────────────────
heading('5.5  Experiment B — Multi-Constellation SPP (Planned)', level=2, size=11, bold=True, color=(40, 80, 140))
body(
    'Experiment B will use the same pipeline and UrbanNav obs file with an extended '
    'navigation dataset covering GPS + BeiDou + Galileo constellations '
    '(RINEX 3 mixed nav from HKSC CORS, DOY 137 / 2021-05-17). '
    'The expected operational difference vs Experiment A:'
)
for item in [
    'Visible satellites: ~9 (GPS) → ~18–22 (GPS+BDS+GAL);',
    'Post-exclusion satellites: ~3 → ~8–12 per epoch;',
    'Valid exclusion epochs: 13% → expected >80%;',
    'Multi-constellation WLS requires per-constellation clock bias: '
    '[X, Y, Z, c·δt_GPS, c·δt_BDS, c·δt_GAL] — 6 unknowns.',
]:
    body('  • ' + item, indent=0.5)
note('[Table 5 — placeholder] Experiment B SPP error comparison once results available. '
     'Format identical to Table 4.')
note('[Fig. 7 — placeholder] Experiment B vs A horizontal error CDF comparison.')
body(
    'If Experiment B shows improvement (consistent with Wen 2019 [5]), '
    'the combined results of A and B will directly support the conclusion that '
    'multi-constellation diversity is a necessary precondition for geometric '
    'ΔL correction to be effective in GPS-only urban canyon scenarios.'
)

# ══════════════════════════════════════════════════════════════════════
# 6. Conclusion and Future Work
# ══════════════════════════════════════════════════════════════════════
heading('6. Conclusion and Future Work')
body(
    'This paper presented a complete, open-source LiDAR–GNSS NLOS pipeline '
    'evaluated on the UrbanNav Medium-Urban-1 dataset. The geometric detector '
    'achieves F1 = 0.574 and estimates multipath extra path lengths with a mean '
    'of 6.70 m, predominantly in the "strong" category (2–10 m, 68%). '
    'A GPS-only SPP correction experiment demonstrates that purely geometric ΔL '
    'correction does not improve positioning under offline-map conditions; '
    'root-cause analysis identifies GPS geometry collapse as the dominant limiting '
    'factor, with mixed-multipath amplitude uncertainty and offline-map normal '
    'reliability as compounding factors.'
)
body('Three directions address the identified limitations:')
for item in [
    'Multi-constellation experiment (Experiment B): downloading RINEX 3 mixed nav '
    '(GPS + BeiDou + Galileo) for the same UrbanNav session is expected to increase '
    'viable post-exclusion epochs from 13% to >80% and validate positive ΔL correction.',
    'Real-time sliding-window LiDAR map: a 250 m sliding window following [4] will '
    'eliminate false NLOS predictions for high-elevation satellites and improve surface '
    'normal reliability by removing accumulated clutter.',
    'SDR-based signal-domain validation: a USRP N210 SDR experiment capturing raw '
    'GNSS IF data will reconstruct DLL correlation functions (61 taps, 0.05-chip '
    'spacing) to directly measure Δτ·c, bridging the geometric-ΔL to tracking-error gap.',
]:
    body('  • ' + item, indent=0.5)

# ══════════════════════════════════════════════════════════════════════
# References
# ══════════════════════════════════════════════════════════════════════
heading('References')
refs = [
    '[1]  Kaplan, E. & Hegarty, C. (2006). Understanding GPS: Principles and Applications, 2nd ed. Artech House.',
    '[2]  Hsu, L.-T. et al. (2021). UrbanNav: An open-source multisensory dataset for benchmarking positioning algorithms. ION GNSS+ 2021.',
    '[3]  Wen, W. et al. (2018). 3D LiDAR aided GNSS NLOS exclusion. ION GNSS+ 2018.',
    '[4]  Wen, W. et al. (2022). 3D LiDAR aided GNSS NLOS mitigation for GNSS-RTK in urban canyons. arXiv:2212.05477.',
    '[5]  Wen, W. et al. (2019). Correcting NLOS by 3D LiDAR and building height to improve GNSS SPP. NAVIGATION, 66(4). doi:10.1002/navi.335.',
    '[6]  del2AINLOS: Double-Difference AI-based NLOS Classification. PSRI-73-2309-PR-Dev, rospak/src/del2AINLOS.',
    '[7]  Groves, P.D. & Adjrad, M. (2017). Likelihood-based GNSS positioning using LOS/NLOS predictions. GPS Solutions, 21(4). doi:10.1007/s10291-017-0654-1.',
    '[8]  Siebler, B. et al. (2023). Coordinate frames and transformations in GNSS ray-tracing for autonomous driving. Remote Sensing, 15(1). doi:10.3390/rs15010180.',
    '[9]  Lau, L. & Cross, P. (2007). Development and testing of a new ray-tracing approach to GNSS carrier-phase multipath modelling. J. Geodesy, 81(11). doi:10.1007/s00190-007-0139-z.',
    '[10] Wen, W. et al. (2021). 3D LiDAR aided GNSS and tightly coupled integration with INS via factor graph. ICRA. arXiv:2106.01594.',
    '[11] Hoppe, H. et al. (1992). Surface reconstruction from unorganized points. SIGGRAPH 1992.',
    '[12] Steingass, A. & Lehner, A. (2004). Measuring the navigation multipath channel — a statistical analysis. ION GNSS 2004.',
    '[13] Groves, P.D. (2013). Principles of GNSS, Inertial, and Multisensor Integrated Navigation, 2nd ed. Artech House.',
    '[14] Braasch, M.S. (1996). Multipath effects. In: Parkinson & Spilker (eds.), Global Positioning System, Vol. 1.',
]
for ref in refs:
    p = doc.add_paragraph()
    p.paragraph_format.left_indent       = Cm(1.0)
    p.paragraph_format.first_line_indent = Cm(-1.0)
    p.paragraph_format.space_after       = Pt(3)
    run = p.add_run(ref)
    set_font(run, size=9.5)

# ══════════════════════════════════════════════════════════════════════
# Appendix A — Pipeline Script Reference
# ══════════════════════════════════════════════════════════════════════
doc.add_page_break()
heading('Appendix A: Pipeline Script Reference', size=12)
body('All scripts are located in gnss_ws/scripts/ on branch claude/review-gnss-spp-JmtgD.', size=10)
add_table(
    ['Step', 'Script', 'Input', 'Output', 'Status'],
    [
        ['Step 2', 'lidar_step2_build_map.py',
         'LiDAR bag', 'urbannav_map.pcd\n(8.53 M pts)', 'Done'],
        ['Step 3', 'lidar_step3_compute_azel.py',
         'RINEX obs + nav\n+ INSPVAX traj', 'epoch_sat_azel.csv\n(4140 records)', 'Done'],
        ['Step 4', 'lidar_step4_ray_casting.py',
         'epoch_sat_azel.csv\n+ urbannav_map.pcd', 'lidar_nlos_prediction.csv\n(hit_dist_m)', 'Done'],
        ['Step 5', 'lidar_step5_compare_labels.py',
         'lidar_nlos_prediction.csv\n+ del2AINLOS labels', 'lidar_nlos_comparison.html', 'Done'],
        ['Step 6', 'lidar_step6_reflection_model.py',
         'lidar_nlos_prediction.csv\n+ urbannav_map.pcd', 'lidar_reflection_model.csv\n+ .html', 'Done'],
        ['Step 7A', 'lidar_step7_spp_correction.py',
         'epoch_sat_azel.csv\n+ reflection_model.csv', 'spp_correction_report.html\n(GPS-only)', 'Done'],
        ['Step 7B', 'lidar_step7_spp_correction.py\n(multi-GNSS nav)',
         'Same + mixed nav file', 'spp_correction_report_multi.html', 'Planned'],
    ],
    caption='Appendix Table A. Pipeline scripts, I/O, and completion status.'
)

# ══════════════════════════════════════════════════════════════════════
# Appendix B — Figure Checklist
# ══════════════════════════════════════════════════════════════════════
heading('Appendix B: Figure Checklist', size=12)
add_table(
    ['ID', 'Content', 'Source file', 'Status'],
    [
        ['Fig. 1', 'Pipeline architecture block diagram', 'Hand-drawn / draw.io',              'To create'],
        ['Fig. 2', 'LiDAR map screenshot (CloudCompare)',  'urbannav_map.pcd',                  'Screenshot ready'],
        ['Fig. 3', 'NLOS detection performance by elevation bin', 'lidar_nlos_comparison.html', 'Generated'],
        ['Fig. 4', 'ΔL distribution + severity donut',    'lidar_reflection_model.html',        'Generated'],
        ['Fig. 5', 'SPP three-mode horizontal error CDF',  'spp_correction_report.html',        'Generated'],
        ['Fig. 6', 'SPP horizontal error time series',     'spp_correction_report.html',        'Generated'],
        ['Fig. 7', 'Exp B vs A horizontal error CDF',      'spp_correction_report_multi.html',  'Planned'],
    ],
    caption='Appendix Table B. Figure checklist.'
)

doc.save(args.out)
print(f'Saved: {args.out}')
