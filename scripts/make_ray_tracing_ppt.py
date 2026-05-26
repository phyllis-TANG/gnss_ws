#!/usr/bin/env python3
"""
make_ray_tracing_ppt.py
Generate a teacher-friendly slide deck (bilingual: zh + en) that explains the
LiDAR ray-tracing block of the GNSS NLOS project.

Outputs:
  docs/ray_tracing_zh.pptx
  docs/ray_tracing_en.pptx

All numbers come from docs/project_report.html (UrbanNav Medium-Urban-1 run).
Uses python-pptx native charts + shapes (editable in PowerPoint / WPS).

Usage:
  pip install python-pptx
  python3 scripts/make_ray_tracing_ppt.py
"""

import os
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LEGEND_POSITION
from pptx.oxml.ns import qn

# ── palette ────────────────────────────────────────────────────────────
PRIMARY = RGBColor(0x1A, 0x23, 0x7E)   # deep indigo
ACCENT  = RGBColor(0x3F, 0x51, 0xB5)
GREEN   = RGBColor(0x0E, 0x9F, 0x6E)
ORANGE  = RGBColor(0xF5, 0x9E, 0x0B)
RED     = RGBColor(0xE0, 0x24, 0x24)
PURPLE  = RGBColor(0x7C, 0x3A, 0xED)
DARK    = RGBColor(0x26, 0x32, 0x38)
MUTED   = RGBColor(0x6B, 0x72, 0x80)
WHITE   = RGBColor(0xFF, 0xFF, 0xFF)
LIGHT   = RGBColor(0xF3, 0xF4, 0xFF)
GREY    = RGBColor(0x9A, 0xA0, 0xA6)
BLDG    = RGBColor(0x90, 0xA4, 0xAE)
BLDG2   = RGBColor(0x60, 0x7D, 0x8B)
SKY     = RGBColor(0xE3, 0xF2, 0xFD)

FONT = {'zh': '微软雅黑', 'en': 'Calibri'}
FONT_EA = '微软雅黑'

EMU_IN = 914400


# ── low-level helpers ──────────────────────────────────────────────────
def _set_ea(run, name):
    """Set East-Asian typeface so Chinese renders with the chosen CJK font."""
    try:
        rPr = run._r.get_or_add_rPr()
        ea = rPr.find(qn('a:ea'))
        if ea is None:
            ea = rPr.makeelement(qn('a:ea'), {})
            rPr.append(ea)
        ea.set('typeface', name)
    except Exception:
        pass


def _style_run(run, size, bold, color, lang):
    f = run.font
    f.size = Pt(size)
    f.bold = bold
    f.color.rgb = color
    f.name = FONT[lang]
    if lang == 'zh':
        _set_ea(run, FONT_EA)


def txt(slide, x, y, w, h, text, size=18, bold=False, color=DARK,
        align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, lang='en', wrap=True):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = wrap
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Pt(2)
    tf.margin_top = tf.margin_bottom = Pt(1)
    p = tf.paragraphs[0]
    p.alignment = align
    r = p.add_run()
    r.text = text
    _style_run(r, size, bold, color, lang)
    return box


def bullets(slide, x, y, w, h, items, size=16, color=DARK, lang='en',
            space=8, leading=1.12):
    """items: list of (text, level) tuples or plain strings (level 0)."""
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.TOP
    first = True
    for it in items:
        if isinstance(it, tuple):
            text, level = it
        else:
            text, level = it, 0
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.space_after = Pt(space)
        p.line_spacing = leading
        p.level = level
        prefix = '•  ' if level == 0 else '–  '
        r = p.add_run()
        r.text = prefix + text
        _style_run(r, size if level == 0 else size - 1, False,
                   color if level == 0 else MUTED, lang)
    return box


def rrect(slide, x, y, w, h, fill, line=None, line_w=1.0, shape=MSO_SHAPE.ROUNDED_RECTANGLE):
    sp = slide.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    sp.fill.solid()
    sp.fill.fore_color.rgb = fill
    if line is None:
        sp.line.fill.background()
    else:
        sp.line.color.rgb = line
        sp.line.width = Pt(line_w)
    sp.shadow.inherit = False
    return sp


def shape_text(sp, lines, lang, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE):
    """lines: list of (text, size, bold, color)."""
    tf = sp.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Pt(4)
    tf.margin_top = tf.margin_bottom = Pt(2)
    first = True
    for text, size, bold, color in lines:
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.alignment = align
        r = p.add_run()
        r.text = text
        _style_run(r, size, bold, color, lang)


def line(slide, x1, y1, x2, y2, color, width=2.0, dash=None, arrow=False):
    conn = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT,
                                      Inches(x1), Inches(y1), Inches(x2), Inches(y2))
    lf = conn.line
    lf.color.rgb = color
    lf.width = Pt(width)
    try:
        ln = lf._get_or_add_ln()
        if dash:
            for el in ln.findall(qn('a:prstDash')):
                ln.remove(el)
            d = ln.makeelement(qn('a:prstDash'), {'val': dash})
            ln.append(d)
        if arrow:
            te = ln.makeelement(qn('a:tailEnd'),
                                {'type': 'triangle', 'w': 'med', 'len': 'med'})
            ln.append(te)
    except Exception:
        pass
    conn.shadow.inherit = False
    return conn


def title_bar(slide, title, lang, page=None, total=None):
    rrect(slide, 0.55, 0.45, 0.32, 0.55, ACCENT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
    txt(slide, 1.05, 0.42, 11.4, 0.7, title, size=27, bold=True, color=PRIMARY,
        anchor=MSO_ANCHOR.MIDDLE, lang=lang)
    line(slide, 0.55, 1.22, 12.78, 1.22, RGBColor(0xDD, 0xDF, 0xEC), width=1.5)
    if page is not None:
        txt(slide, 11.4, 6.95, 1.4, 0.4, f'{page} / {total}', size=10,
            color=MUTED, align=PP_ALIGN.RIGHT, lang=lang)


def add_card(slide, x, y, w, h, number, label, color, lang):
    card = rrect(slide, x, y, w, h, LIGHT, line=color, line_w=1.5)
    shape_text(card, [(number, 22, True, color), (label, 11, False, MUTED)],
               lang, anchor=MSO_ANCHOR.MIDDLE)
    return card


def metric_row(slide, cards, y, h=1.05, x0=0.7, x1=12.63, lang='en'):
    n = len(cards)
    gap = 0.25
    w = (x1 - x0 - gap * (n - 1)) / n
    for i, (num, lab, col) in enumerate(cards):
        add_card(slide, x0 + i * (w + gap), y, w, h, num, lab, col, lang)


def bar_chart(slide, x, y, w, h, cats, name, vals, colors=None, lang='en',
              numfmt='0'):
    cd = CategoryChartData()
    cd.categories = cats
    cd.add_series(name, vals)
    gf = slide.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED,
                                Inches(x), Inches(y), Inches(w), Inches(h), cd)
    ch = gf.chart
    ch.has_legend = False
    ch.has_title = False
    try:
        plot = ch.plots[0]
        plot.gap_width = 80
        plot.has_data_labels = True
        plot.data_labels.number_format = numfmt
        plot.data_labels.number_format_is_linked = False
        plot.data_labels.font.size = Pt(11)
        plot.data_labels.font.bold = True
        plot.data_labels.font.name = FONT[lang]
        series = plot.series[0]
        if colors:
            for idx, pt in enumerate(series.points):
                pt.format.fill.solid()
                pt.format.fill.fore_color.rgb = colors[idx % len(colors)]
    except Exception:
        pass
    try:
        cax = ch.category_axis
        cax.tick_labels.font.size = Pt(11)
        cax.tick_labels.font.name = FONT[lang]
        vax = ch.value_axis
        vax.tick_labels.font.size = Pt(9)
        vax.has_major_gridlines = True
    except Exception:
        pass
    return gf


def donut_chart(slide, x, y, w, h, cats, vals, colors, lang='en'):
    cd = CategoryChartData()
    cd.categories = cats
    cd.add_series('s', vals)
    gf = slide.shapes.add_chart(XL_CHART_TYPE.DOUGHNUT,
                                Inches(x), Inches(y), Inches(w), Inches(h), cd)
    ch = gf.chart
    ch.has_title = False
    ch.has_legend = True
    ch.legend.position = XL_LEGEND_POSITION.RIGHT
    ch.legend.include_in_layout = False
    ch.legend.font.size = Pt(13)
    ch.legend.font.name = FONT[lang]
    try:
        plot = ch.plots[0]
        plot.has_data_labels = True
        plot.data_labels.number_format = '0.0"%"'
        plot.data_labels.number_format_is_linked = False
        plot.data_labels.font.size = Pt(12)
        plot.data_labels.font.bold = True
        plot.data_labels.font.color.rgb = WHITE
        series = plot.series[0]
        for idx, pt in enumerate(series.points):
            pt.format.fill.solid()
            pt.format.fill.fore_color.rgb = colors[idx % len(colors)]
    except Exception:
        pass
    return gf


# ── concept diagrams ───────────────────────────────────────────────────
def star(slide, cx, cy, r, label, lang, color=ORANGE):
    sp = slide.shapes.add_shape(MSO_SHAPE.STAR_5_POINT,
                                Inches(cx - r), Inches(cy - r),
                                Inches(2 * r), Inches(2 * r))
    sp.fill.solid()
    sp.fill.fore_color.rgb = color
    sp.line.fill.background()
    sp.shadow.inherit = False
    txt(slide, cx - 1.0, cy + r - 0.02, 2.0, 0.3, label, size=11, bold=True,
        color=DARK, align=PP_ALIGN.CENTER, lang=lang)


def building(slide, x, y, w, h, color=BLDG, label=None, lang='en'):
    sp = rrect(slide, x, y, w, h, color, shape=MSO_SHAPE.RECTANGLE)
    if label:
        shape_text(sp, [(label, 12, True, WHITE)], lang)
    return sp


def receiver(slide, cx, cy, label, lang):
    sp = slide.shapes.add_shape(MSO_SHAPE.ISOSCELES_TRIANGLE,
                                Inches(cx - 0.16), Inches(cy - 0.16),
                                Inches(0.32), Inches(0.32))
    sp.fill.solid(); sp.fill.fore_color.rgb = PURPLE
    sp.line.fill.background(); sp.shadow.inherit = False
    txt(slide, cx - 0.9, cy + 0.16, 1.8, 0.3, label, size=11, bold=True,
        color=PURPLE, align=PP_ALIGN.CENTER, lang=lang)


def draw_los_nlos(slide, lang, L):
    # sky backdrop
    rrect(slide, 0.6, 1.45, 12.18, 5.05, SKY, shape=MSO_SHAPE.RECTANGLE)
    gy = 6.35
    line(slide, 0.6, gy, 12.78, gy, RGBColor(0x8D, 0x6E, 0x63), width=3)
    # buildings
    building(slide, 2.3, 2.5, 1.5, gy - 2.5, BLDG, L['bldg'], lang)
    building(slide, 9.3, 1.95, 1.7, gy - 1.95, BLDG2, L['bldg'], lang)
    # receiver
    rx = (6.6, gy - 0.18)
    receiver(slide, rx[0], rx[1], L['rx'], lang)
    # LOS satellite (clear sky gap)
    los_sat = (5.1, 2.05)
    star(slide, los_sat[0], los_sat[1], 0.32, L['sat_los'], lang, GREEN)
    line(slide, rx[0], rx[1], los_sat[0], los_sat[1], GREEN, width=3, arrow=True)
    txt(slide, 5.0, 4.0, 2.1, 0.35, L['los'], size=13, bold=True, color=GREEN,
        align=PP_ALIGN.CENTER, lang=lang)
    # NLOS satellite (top-right, behind right building)
    nlos_sat = (11.9, 1.45)
    star(slide, nlos_sat[0], nlos_sat[1], 0.32, L['sat_nlos'], lang, RED)
    # blocked direct path
    line(slide, rx[0], rx[1], nlos_sat[0], nlos_sat[1], RED, width=2.2, dash='dash')
    txt(slide, 6.9, 4.25, 2.4, 0.32, '✗ ' + L['blocked'], size=12, bold=True,
        color=RED, align=PP_ALIGN.CENTER, lang=lang)
    # reflected path nlos_sat -> wall P -> rx
    P = (3.8, 3.35)
    line(slide, nlos_sat[0], nlos_sat[1], P[0], P[1], ORANGE, width=2.6)
    line(slide, P[0], P[1], rx[0], rx[1], ORANGE, width=2.6, arrow=True)
    txt(slide, 3.9, 4.55, 3.0, 0.6, L['reflected'], size=12, bold=True,
        color=ORANGE, align=PP_ALIGN.LEFT, lang=lang)


def draw_stepping(slide, lang, L):
    rrect(slide, 0.6, 1.5, 12.18, 4.9, SKY, shape=MSO_SHAPE.RECTANGLE)
    gy = 6.25
    line(slide, 0.6, gy, 12.78, gy, RGBColor(0x8D, 0x6E, 0x63), width=3)
    building(slide, 9.6, 2.1, 1.9, gy - 2.1, BLDG2, L['bldg'], lang)
    rx = (1.7, gy - 0.18)
    receiver(slide, rx[0], rx[1], L['rx'], lang)
    sat = (12.0, 1.55)
    star(slide, sat[0], sat[1], 0.30, L['sat'], lang, ORANGE)
    # full intended ray (light)
    line(slide, rx[0], rx[1], sat[0], sat[1], RGBColor(0xCF, 0xD8, 0xDC), width=1.5, dash='sysDot')
    # stepping squares from rx toward sat until building hit (~x=9.6)
    import math
    dx, dy = sat[0] - rx[0], sat[1] - rx[1]
    dist = math.hypot(dx, dy)
    ux, uy = dx / dist, dy / dist
    step = 0.62
    s = 0.18
    hit_done = False
    d = 0.7
    while d < dist:
        px = rx[0] + ux * d
        py = rx[1] + uy * d
        in_bldg = (9.6 <= px <= 11.5) and (py >= 2.1)
        col = RED if in_bldg else GREEN
        sq = rrect(slide, px - s / 2, py - s / 2, s, s, col, shape=MSO_SHAPE.RECTANGLE)
        if in_bldg and not hit_done:
            hit_done = True
            txt(slide, px - 1.5, py - 0.95, 3.0, 0.4, '✗ ' + L['hit'], size=13,
                bold=True, color=RED, align=PP_ALIGN.CENTER, lang=lang)
            break
        d += step
    txt(slide, 2.0, 5.4, 6.5, 0.5, L['step_note'], size=13, bold=True,
        color=ACCENT, align=PP_ALIGN.LEFT, lang=lang)


def draw_2bounce(slide, lang, L):
    rrect(slide, 0.6, 1.5, 12.18, 4.95, SKY, shape=MSO_SHAPE.RECTANGLE)
    gy = 6.3
    line(slide, 0.6, gy, 12.78, gy, RGBColor(0x8D, 0x6E, 0x63), width=3)
    building(slide, 1.8, 2.0, 1.5, gy - 2.0, BLDG, L['bldg'], lang)
    building(slide, 10.0, 2.0, 1.5, gy - 2.0, BLDG2, L['bldg'], lang)
    rx = (6.65, gy - 0.18)
    receiver(slide, rx[0], rx[1], L['rx'], lang)
    # outgoing ray (matches algorithm: shot from rx toward satellite direction)
    P1 = (10.0, 3.7)   # first hit, right building inner wall
    P2 = (3.3, 2.95)   # second hit, left building inner wall
    line(slide, rx[0], rx[1], P1[0], P1[1], ORANGE, width=2.8, arrow=True)
    line(slide, P1[0], P1[1], P2[0], P2[1], ORANGE, width=2.8, arrow=True)
    star(slide, 1.5, 1.75, 0.26, L['sat'], lang, ORANGE)
    line(slide, P2[0], P2[1], 1.7, 1.95, RGBColor(0xCF, 0xD8, 0xDC), width=1.5, dash='sysDot')
    # labels for segments
    txt(slide, 7.6, 4.7, 2.3, 0.35, 'd1', size=15, bold=True, color=RED,
        align=PP_ALIGN.CENTER, lang=lang)
    txt(slide, 5.6, 2.7, 2.3, 0.35, 'd2', size=15, bold=True, color=RED,
        align=PP_ALIGN.CENTER, lang=lang)
    # normal arrows
    line(slide, P1[0], P1[1], P1[0] - 0.7, P1[1] - 0.15, PURPLE, width=2, arrow=True)
    line(slide, P2[0], P2[1], P2[0] + 0.7, P2[1] + 0.1, PURPLE, width=2, arrow=True)
    txt(slide, 2.0, 5.35, 9.5, 0.5, L['b2_note'], size=13, bold=True,
        color=ACCENT, align=PP_ALIGN.CENTER, lang=lang)


def draw_pipeline(slide, steps, lang, y=2.0, h=1.6):
    n = len(steps)
    x0, x1 = 0.7, 12.63
    gap = 0.5
    w = (x1 - x0 - gap * (n - 1)) / n
    cols = [ACCENT, PURPLE, RED, ORANGE, GREEN]
    for i, s in enumerate(steps):
        x = x0 + i * (w + gap)
        box = rrect(slide, x, y, w, h, LIGHT, line=cols[i % len(cols)], line_w=2)
        shape_text(box, [(f'{i+1}', 18, True, cols[i % len(cols)]),
                         (s, 12.5, True, DARK)], lang)
        if i < n - 1:
            line(slide, x + w + 0.05, y + h / 2, x + w + gap - 0.05, y + h / 2,
                 GREY, width=2.5, arrow=True)


def formula_box(slide, x, y, w, h, text, lang):
    box = rrect(slide, x, y, w, h, RGBColor(0xF8, 0xF9, 0xFF), line=ACCENT, line_w=1.5)
    shape_text(box, [(text, 22, True, PRIMARY)], lang)
    return box


# ── translations ───────────────────────────────────────────────────────
COMMON = dict(
    bldg=dict(zh='楼', en='Building'),
    rx=dict(zh='接收机', en='Receiver'),
    sat=dict(zh='卫星', en='Satellite'),
    sat_los=dict(zh='卫星A', en='Sat A'),
    sat_nlos=dict(zh='卫星B', en='Sat B'),
    los=dict(zh='直射 LOS', en='Direct (LOS)'),
    blocked=dict(zh='被楼挡住', en='blocked'),
    reflected=dict(zh='反射到达 (NLOS)\n多走了 ΔL', en='reflected (NLOS)\nextra path ΔL'),
    hit=dict(zh='撞到楼 → NLOS', en='hits building → NLOS'),
    step_note=dict(zh='每 0.5m 前进一步，逐个小方块检查有没有建筑',
                   en='Step 0.5 m at a time, check each voxel for a building'),
    b2_note=dict(zh='信号拐两次弯：撞墙1 (d1) → 反射 → 撞墙2 (d2) → 到接收机，总额外路径 ≈ d1 + d2',
                 en='Two bounces: wall 1 (d1) → reflect → wall 2 (d2) → receiver,  total extra path ≈ d1 + d2'),
)


def C(key, lang):
    return COMMON[key][lang]


# ── deck builder ───────────────────────────────────────────────────────
def build(lang):
    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)
    blank = prs.slide_layouts[6]
    L = {k: C(k, lang) for k in COMMON}
    total = 13

    def new():
        return prs.slides.add_slide(blank)

    zh = (lang == 'zh')

    # ── S1 title ──
    s = new()
    rrect(s, 0, 0, 13.333, 7.5, PRIMARY, shape=MSO_SHAPE.RECTANGLE)
    rrect(s, 0, 5.7, 13.333, 1.8, RGBColor(0x12, 0x19, 0x5E), shape=MSO_SHAPE.RECTANGLE)
    txt(s, 0.9, 2.1, 11.5, 1.4,
        'LiDAR 射线追踪辅助 GNSS NLOS 检测' if zh else
        'LiDAR Ray Tracing for GNSS NLOS Detection',
        size=40, bold=True, color=WHITE, lang=lang)
    txt(s, 0.9, 3.6, 11.5, 1.2,
        '把 3D 点云当作“城市”，判断每颗卫星信号是直射还是被楼挡住' if zh else
        'Use the 3D point cloud as the “city” to tell whether each satellite signal is direct or blocked',
        size=19, color=RGBColor(0xC5, 0xCA, 0xE9), lang=lang)
    txt(s, 0.9, 5.95, 11.5, 1.2,
        'UrbanNav HK Medium-Urban-1 · 香港西九龙 · 2021-05-17 · ~657 历元' if zh else
        'UrbanNav HK Medium-Urban-1 · West Kowloon, Hong Kong · 2021-05-17 · ~657 epochs',
        size=14, color=RGBColor(0x9F, 0xA8, 0xDA), lang=lang)

    # ── S2 problem ──
    s = new()
    title_bar(s, '问题：城市峡谷里，卫星信号会被楼挡住' if zh else
              'The problem: in urban canyons, buildings block satellite signals',
              lang, 2, total)
    bullets(s, 0.7, 1.4, 12.0, 1.4, [
        ('香港高楼林立，很多卫星信号无法直达接收机' if zh else
         'In dense high-rise areas, many satellite signals cannot reach the receiver directly', 0),
        ('信号被楼挡住后经墙面反射再到达 — 这叫 NLOS（非直射）' if zh else
         'A blocked signal arrives only after reflecting off a wall — this is called NLOS (non-line-of-sight)', 0),
        ('反射让信号多走一段路 → 测距偏长 → 定位偏差可达几十米' if zh else
         'Reflection adds extra travel distance → pseudorange too long → positioning error of tens of metres', 0),
    ], size=16, lang=lang)
    draw_los_nlos(s, lang, L)

    # ── S3 idea ──
    s = new()
    title_bar(s, '核心思路：用 LiDAR 地图给每颗卫星“连连看”' if zh else
              'Core idea: ray-trace each satellite through a LiDAR map',
              lang, 3, total)
    bullets(s, 0.9, 1.7, 11.5, 4.5, [
        ('LiDAR 扫描得到的 3D 点云 = 这座城市的精确模型（含每一栋楼）' if zh else
         'The 3D point cloud from LiDAR is a precise model of the city (every building included)', 0),
        ('从接收机朝卫星方向画一条直线（射线）' if zh else
         'From the receiver, draw a straight line (a ray) toward each satellite', 0),
        ('射线一路畅通 → LOS（直射）；中途撞到楼 → NLOS（被挡）' if zh else
         'Ray reaches the sky unobstructed → LOS (direct);  ray hits a building → NLOS (blocked)', 0),
        ('纯几何判断，不需要训练数据，又快又可解释' if zh else
         'Pure geometry — no training data needed, fast and fully explainable', 0),
        ('好处：知道哪些卫星不可信，就能在定位时区别对待' if zh else
         'Payoff: knowing which satellites are unreliable lets us treat them differently when positioning', 0),
    ], size=18, lang=lang, space=14)

    # ── S4 pipeline ──
    s = new()
    title_bar(s, '整体流程：五步走' if zh else 'The full pipeline: five steps',
              lang, 4, total)
    steps = (['LiDAR 点云地图', '算卫星方向\n方位/仰角', '射线投射\n判 LOS/NLOS',
              '反射几何\n估算 ΔL', '送入定位\n做改正'] if zh else
             ['LiDAR point-cloud map', 'Satellite directions\n(az / el)',
              'Ray casting\nLOS / NLOS', 'Reflection geometry\nestimate ΔL',
              'Feed into\npositioning'])
    draw_pipeline(s, steps, lang, y=2.6, h=1.9)
    txt(s, 0.7, 5.1, 12.0, 1.2,
        '本次汇报聚焦中间三步（红/橙框）——射线追踪如何判 NLOS、如何把“多走的路”算出来。' if zh else
        'This talk focuses on the middle three steps (red/orange): how ray tracing finds NLOS and estimates the extra path.',
        size=15, color=MUTED, lang=lang)

    # ── S5 build map ──
    s = new()
    title_bar(s, '第一步：把点云变成“积木世界”（体素地图）' if zh else
              'Step 1: turn the point cloud into a voxel world',
              lang, 5, total)
    bullets(s, 0.7, 1.45, 12.0, 1.7, [
        ('体素 = 把空间切成 0.5m 的小方块；方块里有点 = 这里有建筑' if zh else
         'A voxel = a 0.5 m cube of space; a cube containing points = a building is there', 0),
        ('先过滤掉地面点（只保留接收机高度以上），避免误判' if zh else
         'Ground points are filtered out (keep only what is above the receiver) to avoid false hits', 0),
        ('这样射线只要查“方块占用没占用”，速度极快' if zh else
         'The ray only needs to ask “is this cube occupied?”, which is extremely fast', 0),
    ], size=16, lang=lang)
    metric_row(s, [
        ('8,531,577', '点云总点数' if zh else 'total cloud points', ACCENT),
        ('2,860,032', '过滤地面后' if zh else 'after ground filter', PURPLE),
        ('0.5 m', '体素大小' if zh else 'voxel size', ORANGE),
        ('1,333,519', '占用体素数' if zh else 'occupied voxels', GREEN),
    ], y=4.5, lang=lang)

    # ── S6 sat directions ──
    s = new()
    title_bar(s, '第二步：算出每颗卫星在天上的方向' if zh else
              'Step 2: compute where each satellite is in the sky',
              lang, 6, total)
    bullets(s, 0.7, 1.45, 12.0, 1.9, [
        ('逐历元、逐卫星计算方位角 + 仰角（卫星在天空哪个位置）' if zh else
         'For every epoch and satellite, compute azimuth + elevation (its position in the sky)', 0),
        ('三大星座一起用：GPS + 北斗 BeiDou + 伽利略 Galileo' if zh else
         'Three constellations together: GPS + BeiDou + Galileo', 0),
        ('星座越多 → 可见卫星越多 → 几何越好 → 定位越稳' if zh else
         'More constellations → more visible satellites → better geometry → more robust positioning', 0),
    ], size=16, lang=lang)
    metric_row(s, [
        ('5.8', 'GPS 颗/历元' if zh else 'GPS sats/epoch', ACCENT),
        ('6.4', '北斗 颗/历元' if zh else 'BeiDou sats/epoch', RED),
        ('3.8', '伽利略 颗/历元' if zh else 'Galileo sats/epoch', GREEN),
        ('15.9', '平均总数/历元' if zh else 'avg total/epoch', PURPLE),
    ], y=4.6, lang=lang)

    # ── S7 ray casting ──
    s = new()
    title_bar(s, '第三步：发射射线，逐格检查有没有撞楼' if zh else
              'Step 3: cast the ray and check voxel by voxel',
              lang, 7, total)
    bullets(s, 0.7, 1.4, 12.0, 1.0, [
        ('沿卫星方向每 0.5m 前进一步，查这个小方块是不是建筑；80m 内撞到 → NLOS，否则 LOS' if zh else
         'Step 0.5 m along the satellite direction, test each cube; a hit within 80 m → NLOS, otherwise LOS', 0),
    ], size=15, lang=lang)
    draw_stepping(s, lang, L)
    txt(s, 8.6, 5.95, 4.1, 0.5,
        '处理速度 14,773 条/秒（全程约 1 秒）' if zh else
        'Throughput: 14,773 rays/s (~1 s total)',
        size=13, bold=True, color=GREEN, align=PP_ALIGN.RIGHT, lang=lang)

    # ── S8 NLOS result ──
    s = new()
    title_bar(s, '结果：超过一半卫星是 NLOS' if zh else
              'Result: more than half of all satellites are NLOS',
              lang, 8, total)
    donut_chart(s, 0.8, 1.7, 6.0, 4.6,
                ['LOS', 'NLOS'], [4472, 6578], [GREEN, RED], lang=lang)
    bullets(s, 7.2, 2.2, 5.5, 4.0, [
        ('总计 11,050 条卫星×历元' if zh else 'Total 11,050 satellite×epoch records', 0),
        ('NLOS 6,578 条 = 59.5%' if zh else 'NLOS: 6,578 = 59.5%', 0),
        ('平均每历元 15.9 颗可见，约 9.5 颗被挡' if zh else
         '~9.5 of the 15.9 visible satellites per epoch are blocked', 0),
        ('直接用全部伪距定位，必然受严重多径污染' if zh else
         'Using all pseudoranges blindly is bound to suffer heavy multipath', 0),
    ], size=17, lang=lang, space=14)

    # ── S9 2-bounce ──
    s = new()
    title_bar(s, '进阶：不止一次反射 — 二次反射建模' if zh else
              'Going further: modelling double reflections (2-bounce)',
              lang, 9, total)
    bullets(s, 0.7, 1.35, 12.0, 1.1, [
        ('射线撞墙后，用周围点云算出墙面朝向（法向量，PCA），再沿镜面反射方向继续追踪第二面墙' if zh else
         'After a hit, estimate the wall orientation (normal via PCA) and trace on along the mirror-reflection direction to a second wall', 0),
    ], size=15, lang=lang)
    draw_2bounce(s, lang, L)

    # ── S10 reflection ΔL ──
    s = new()
    title_bar(s, '把“多走的路”算出来：反射几何 ΔL' if zh else
              'Quantifying the extra path: reflection geometry ΔL',
              lang, 10, total)
    formula_box(s, 0.8, 1.45, 5.4, 1.0, 'ΔL = 2 · d · cos θ', lang)
    bullets(s, 6.5, 1.4, 6.2, 1.5, [
        ('d = 撞墙距离，θ = 入射角（由法向量算出）' if zh else
         'd = distance to the wall, θ = incidence angle (from the normal)', 0),
        ('ΔL = 信号比直线多走的距离' if zh else
         'ΔL = how much further the signal travels than a straight line', 0),
    ], size=14, lang=lang)
    metric_row(s, [
        ('2,753', '成功建模 NLOS' if zh else 'NLOS modelled', ACCENT),
        ('6.70 m', 'ΔL 均值' if zh else 'mean ΔL', ORANGE),
        ('6.45 m', 'ΔL 中位数' if zh else 'median ΔL', GREEN),
        ('54.6°', '平均入射角' if zh else 'mean incidence', PURPLE),
    ], y=2.75, h=0.95, lang=lang)
    bar_chart(s, 1.2, 4.0, 7.0, 2.9,
              (['轻度 <2m', '强反射 2-10m', '严重 ≥10m'] if zh else
               ['Mild <2m', 'Strong 2-10m', 'Severe ≥10m']),
              'ΔL', [402, 1868, 483], [GREEN, ORANGE, RED], lang=lang)
    txt(s, 8.4, 4.2, 4.3, 2.5,
        ('多径严重程度分布：\n大多数 NLOS 属“强反射”\n（高楼幕墙），均值约 6.7m' if zh else
         'Multipath severity:\nmost NLOS are “strong”\n(glass facades), ~6.7 m on average'),
        size=14, color=MUTED, lang=lang)

    # ── S11 application ──
    s = new()
    title_bar(s, '这有什么用：把 ΔL 还给定位' if zh else
              'What it is for: give ΔL back to positioning',
              lang, 11, total)
    bullets(s, 0.7, 1.4, 12.0, 1.3, [
        ('NLOS 卫星的伪距减去 ΔL → 修正多径误差；不可靠的还可降权或剔除' if zh else
         'Subtract ΔL from the NLOS pseudorange to correct multipath; unreliable ones can be down-weighted or removed', 0),
        ('三种定位模式对比（水平均值误差）：' if zh else
         'Three positioning modes compared (mean horizontal error):', 0),
    ], size=15, lang=lang)
    bar_chart(s, 1.2, 3.0, 7.2, 3.4,
              (['基准\nbaseline', '剔除\nexclusion', '改正\ncorrection'] if zh else
               ['Baseline', 'Exclusion', 'Correction']),
              'm', [26.45, 99.00, 39.97], [GREEN, RED, ORANGE], lang=lang,
              numfmt='0.0')
    bullets(s, 8.7, 3.1, 4.0, 3.4, [
        ('剔除全部 NLOS → 卫星不够用，反而更差（99m）' if zh else
         'Removing all NLOS starves geometry — worse (99 m)', 0),
        ('改正目前 39.97m，仍未超过 baseline 26.45m' if zh else
         'Correction is 39.97 m, not yet beating baseline 26.45 m', 0),
        ('为什么？见下一页' if zh else 'Why? See next slide', 0),
    ], size=14, lang=lang, color=DARK)

    # ── S12 limitations & next ──
    s = new()
    title_bar(s, '目前的局限 & 下一步' if zh else 'Current limits & next steps',
              lang, 12, total)
    c1 = rrect(s, 0.7, 1.5, 5.85, 4.8, RGBColor(0xFE, 0xF2, 0xF2), line=RED, line_w=1.5)
    txt(s, 0.95, 1.65, 5.4, 0.5, '局限' if zh else 'Limitations', size=18, bold=True,
        color=RED, lang=lang)
    bullets(s, 0.95, 2.3, 5.4, 3.9, [
        ('建模覆盖率仅 42%（3,825 条 NLOS 因时间戳没对上被跳过）' if zh else
         'Only 42% of NLOS modelled (3,825 skipped due to timestamp mismatch)', 0),
        ('41% 的反射面是墙角/曲面，法向量不可靠，ΔL 偏差 5–15m' if zh else
         '41% of surfaces are corners/curved — unreliable normals, ΔL off by 5–15 m', 0),
        ('改正太“一刀切”，没按置信度加权' if zh else
         'Correction is applied bluntly, with no confidence weighting', 0),
    ], size=14, lang=lang, space=10)
    c2 = rrect(s, 6.85, 1.5, 5.85, 4.8, RGBColor(0xF0, 0xFD, 0xF4), line=GREEN, line_w=1.5)
    txt(s, 7.1, 1.65, 5.4, 0.5, '下一步' if zh else 'Next steps', size=18, bold=True,
        color=GREEN, lang=lang)
    bullets(s, 7.1, 2.3, 5.4, 3.9, [
        ('修复时间戳/闰秒对齐 → 建模覆盖率 42% → >85%' if zh else
         'Fix timestamp/leap-second alignment → coverage 42% → >85%', 0),
        ('按法向量置信度做“软改正”，平面性低就少改' if zh else
         'Confidence-weighted “soft” correction (trust flat surfaces more)', 0),
        ('加入故障检测剔除（FDE / RAIM），去掉漏判的大误差卫星' if zh else
         'Add fault detection (FDE / RAIM) to drop missed large-error satellites', 0),
        ('目标：correction RMS 降到 20–25m 以下，真正超过 baseline' if zh else
         'Goal: push correction RMS below 20–25 m to truly beat baseline', 0),
    ], size=14, lang=lang, space=10)

    # ── S13 summary ──
    s = new()
    rrect(s, 0, 0, 13.333, 7.5, PRIMARY, shape=MSO_SHAPE.RECTANGLE)
    txt(s, 0.9, 0.7, 11.5, 0.9, '一页总结' if zh else 'One-slide summary',
        size=30, bold=True, color=WHITE, lang=lang)
    line(s, 0.9, 1.65, 12.4, 1.65, RGBColor(0x5C, 0x6B, 0xC0), width=1.5)
    bullets(s, 1.0, 2.0, 11.4, 4.6, [
        ('跑通端到端射线追踪流水线：LiDAR 地图 → 卫星方向 → LOS/NLOS → 反射 ΔL → 定位' if zh else
         'End-to-end ray-tracing pipeline: LiDAR map → sat directions → LOS/NLOS → reflection ΔL → positioning', 0),
        ('量化香港城市峡谷 NLOS 比例：59.5%（11,050 条中 6,578 条）' if zh else
         'Quantified the urban-canyon NLOS rate: 59.5% (6,578 of 11,050)', 0),
        ('几何建模反射多径：ΔL 均值 6.7m，纯几何、可解释、14,773 条/秒' if zh else
         'Geometrically modelled multipath: mean ΔL 6.7 m, explainable, 14,773 rays/s', 0),
        ('多星座 baseline 已达 26.45m，城市峡谷里属较好水平' if zh else
         'Multi-constellation baseline reaches 26.45 m — already good for an urban canyon', 0),
        ('下一步把改正做出正收益（提覆盖率 + 置信度加权 + FDE）' if zh else
         'Next: make the correction a net win (coverage + confidence weighting + FDE)', 0),
    ], size=18, lang=lang, color=WHITE, space=16)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'docs', f'ray_tracing_{lang}.pptx')
    out = os.path.abspath(out)
    prs.save(out)
    print('saved:', out)
    return out


if __name__ == '__main__':
    build('zh')
    build('en')
