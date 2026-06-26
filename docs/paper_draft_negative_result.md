# Single-Epoch LiDAR Reflectance Features Do Not Generalizably Discriminate GNSS NLOS Pseudorange Errors Beyond Satellite Elevation

**A Negative Result with a Cross-Validation Leakage Caution for GNSS–LiDAR Studies**

> 论文草稿 v0.1 — 所有数值取自本项目 UrbanNav-HK-Medium-Urban-1 实测结果（step6e/6f/6g/6h/8d/8e/8f/8g/8h）。
> 标 `[TODO]` 处需补图或补文献。

---

## 摘要 / Abstract（中文）

城市峡谷中非视距（NLOS）信号是 GNSS 定位误差的主因。近年大量工作尝试用车载
LiDAR 提供的三维结构与反射率信息来预测、剔除或改正 NLOS 观测。本文以一个香港
城市峡谷数据集（UrbanNav Medium-Urban-1，F9P 多星座接收机 + 同步 LiDAR）系统检验
一个看似自然的命题：**单历元 LiDAR 反射率/几何特征能否独立判别 GNSS 伪距域 NLOS
误差**。我们构建了归一化反射率 ρ_norm（含距离平方与入射角补偿）、基于香港 CORS
参考站 HKSC 的双差残差作为干净的 NLOS 真值标签，并设计了一套逐层剥离混淆变量的
诊断流程。结果表明：(1) 反射率与 C/N0 衰减仅在 GPS 严重遮挡子集中呈弱显著负相关
（Spearman r=−0.167，95% CI 排除 0），多星座扩展后整体相关性消失；(2) LiDAR 几何
遮挡标签与双差实测 NLOS 仅 F1=0.27 一致，83% 的 LiDAR-NLOS 卫星实测无伪距偏差；
(3) 一个表面性能很高的分类器（AUC=0.906）其判别力几乎完全来自卫星仰角，加入
LiDAR 特征的边际增益约为零；(4) 剥离仰角后残差 LiDAR 特征在随机交叉验证下仍显
AUC=0.748，但这是**时序自相关泄漏**所致——在按卫星分组的交叉验证下塌回 0.533
（≈随机）。我们因此得出严谨的阴性结论，并指出 GNSS–LiDAR/机器学习研究中一个
普遍存在却少被报告的随机交叉验证泄漏陷阱。

**关键词**：GNSS NLOS、LiDAR 反射率、城市峡谷、双差、交叉验证泄漏、阴性结果

## Abstract（English）

Non-line-of-sight (NLOS) reception is the dominant source of GNSS positioning
error in urban canyons. A growing body of work uses vehicle-borne LiDAR — its
3-D structure and surface reflectance — to predict, exclude, or correct NLOS
measurements. Using a Hong Kong urban-canyon dataset (UrbanNav Medium-Urban-1;
u-blox F9P multi-constellation receiver with synchronized LiDAR), we rigorously
test a seemingly natural hypothesis: **can single-epoch LiDAR reflectance and
geometric features independently discriminate GNSS pseudorange-domain NLOS
errors?** We construct a normalized reflectance ρ_norm (range² and
incidence-angle compensated), derive clean NLOS truth labels from
double-difference (DD) residuals against the Hong Kong CORS reference station
HKSC, and apply a diagnostic protocol that strips confounders layer by layer.
We find: (1) reflectance correlates with C/N0 attenuation only weakly and only
in the severely-occluded GPS subset (Spearman r=−0.167, 95% CI excludes 0),
with the correlation vanishing once the pool is expanded to multiple
constellations; (2) LiDAR geometric-occlusion labels agree with DD-measured
NLOS at only F1=0.27, and 83% of LiDAR-flagged NLOS satellites show no measured
pseudorange bias; (3) a classifier with high apparent performance (AUC=0.906)
derives its discrimination almost entirely from satellite elevation, with the
marginal gain from LiDAR features being essentially zero; (4) after
residualizing elevation, LiDAR features still show AUC=0.748 under random
cross-validation, but this is an artifact of **temporal autocorrelation
leakage** — under satellite-grouped cross-validation it collapses to 0.533
(≈chance). We therefore report a rigorous negative result and highlight a
common but under-reported random-CV leakage pitfall in GNSS–LiDAR / machine
learning studies.

---

## 1. Introduction

- GNSS in urban canyons: NLOS and multipath dominate error budget; our own SPP
  baseline on this dataset shows mean error 84.8 m, RMS 103.8 m, 95th percentile
  176.1 m — consistent with the 50–200 m expected for GNSS-only SPP in dense
  Hong Kong canyons. [TODO: cite UrbanNav, 3DMA-GNSS surveys]
- Motivation for LiDAR aiding: LiDAR gives dense 3-D structure (which directions
  are blocked) and **reflectance** (material properties of reflecting surfaces).
  The intuitive chain is: glass/metal façades → strong specular reflection →
  NLOS path → biased pseudorange and/or attenuated C/N0. If LiDAR reflectance
  encodes this, it should predict which satellites are NLOS and by how much.
- Gap / our question: most prior LiDAR-aided NLOS work uses **geometry** (ray
  tracing / shadow matching) and reports good classification scores. Far less
  work isolates whether the **reflectance/material** signal carries *independent*
  predictive information, and almost none audits the cross-validation protocol
  for temporal leakage. We ask the narrow, falsifiable question above and answer
  it honestly.
- Contributions:
  1. A reproducible pipeline normalizing raw LiDAR intensity to a physically
     meaningful reflectance proxy ρ_norm and aligning it per-epoch-per-satellite
     with GNSS observables.
  2. A clean NLOS truth label from double-difference residuals (receiver and
     satellite clocks cancel) that avoids the urban-canyon clock-estimation
     failure we document.
  3. A layered diagnostic establishing that single-epoch LiDAR features provide
     **no generalizable independent discrimination** of pseudorange NLOS beyond
     satellite elevation.
  4. A concrete demonstration and quantification of **random-CV leakage**
     (AUC 0.748 → 0.533) — a methodological caution for the field.

## 2. Related Work

[TODO: fill citations — group into]
- 3-D map / ray-tracing NLOS exclusion (shadow matching, 3DMA-GNSS).
- LiDAR / camera fisheye NLOS detection (sky segmentation).
- Machine-learning NLOS classification from CN0/elevation/pseudorange features
  (note: many report high AUC/F1; flag those using random train/test splits on
  time-series epochs).
- LiDAR reflectance/intensity calibration (range and incidence-angle
  normalization) literature.

## 3. Data and Methods

### 3.1 Dataset
- UrbanNav-HK-Medium-Urban-1, Hong Kong, 2021-05-17, ~13 min, 657 epochs.
- Receiver: u-blox F9P, recording GPS (G), BeiDou (C), Galileo (E).
- Reference truth: UrbanNav RTK/INS ground truth (787 points).
- Reference station for DD: Hong Kong CORS **HKSC** (RINEX nav hksc137c.21n).
- Synchronized LiDAR point cloud with per-return intensity. [TODO: sensor model,
  rate, extrinsics source]

### 3.2 Normalized LiDAR reflectance ρ_norm
Raw intensity I depends on range R, incidence angle α, and surface reflectance.
We compensate the dominant geometric factors:

  ρ_norm = I · R² / (η_ref · cos α)

then median-normalize ρ_norm to 1.0 across the dataset. η_ref is a sensor
reference constant. [TODO: state η_ref handling and any saturation clipping.]
This yields a per-return material proxy approximately independent of range and
local geometry.

### 3.3 C/N0 attenuation metric
For each constellation separately (G/C/E have different C/N0 baselines), we fit
an expected C/N0 as a 2nd-degree polynomial in elevation, with a per-satellite
bias correction, then define

  CN0_drop = CN0_expected(elev, sat) − CN0_measured.

### 3.4 Double-difference NLOS labels
Single-receiver pseudorange error estimation failed in this canyon: the
per-constellation LOS-satellite clock estimate was corrupted by the high NLOS
fraction (negative, physically impossible psr_error in 5th percentile; clock
solution failed in 402/657 epochs). We instead form double differences against
HKSC, cancelling receiver and satellite clocks. The DD residual dd_resid then
approximates the extra NLOS path length ΔL. After DD, the 5th percentile of
dd_resid is −9.1 m (clock pathology resolved). We label a satellite-epoch as
measured NLOS when |dd_resid| > 20 m and as clean when |dd_resid| < 5 m,
discarding the 5–20 m ambiguous band.

### 3.5 LiDAR geometric NLOS and features
LiDAR ray casting flags a satellite as geometrically NLOS when the line of
sight is occluded; the reflecting facet gives an extra path length ΔL
(delta_L), incidence angle, and ρ_norm. These are the candidate LiDAR features,
joined per epoch-per-satellite to dd_resid.

### 3.6 Diagnostic protocol
A sequence of falsification tests (scripts step6e–step8h):
1. Bootstrap CI + stratified + partial correlation of ρ_norm vs CN0_drop.
2. Multi-constellation expansion and per-constellation severe-NLOS
   cross-validation.
3. BeiDou orbit-type confounding check (GEO/IGSO/MEO).
4. DD-residual material/geometry correlation; LiDAR-vs-DD agreement (F1).
5. AUC of LiDAR features discriminating DD-measured NLOS.
6. Ablation: elevation-only vs LiDAR-only vs combined (marginal contribution).
7. Elevation residualization (proxy vs independent signal).
8. **Grouped cross-validation** (random vs by-satellite vs by-time-block) to
   detect autocorrelation leakage.

## 4. Results

### 4.1 Reflectance vs C/N0 attenuation is weak and condition-specific
- GPS-only (n≈2100): Spearman r=−0.058, 95% bootstrap CI [−0.099, −0.017]
  (excludes 0); 8/8 strata negative; partial r=−0.077 (p=0.0004) controlling
  elevation and hit distance. A real but very weak effect.
- Multi-constellation (n=5421): r=+0.006, CI [−0.020, +0.032] (includes 0). The
  effect dilutes. Galileo CN0_drop averages −0.06 dB — its E1 BOC(1,1)
  modulation is largely immune to NLOS C/N0 degradation.
- Severe-NLOS per constellation (step6g): only **GPS** is robustly negative
  (r=−0.167, CI [−0.264, −0.071], 99.9% of bootstrap samples negative). BeiDou
  severe r=+0.063 (CI includes 0); Galileo severe r=−0.119 (CI [−0.263, +0.025],
  95% negative).
- BeiDou caveat (step6h): the BeiDou subset is IGSO-dominated (86%, 1640/1909);
  no GEO present; only C27/C30 are MEO (severe MEO n=30, too few). BeiDou's null
  is therefore inconclusive — recorded as a limitation, not evidence of absence.

> Takeaway: a material→attenuation effect exists but is detectable only under
> severe GPS occlusion; it is too weak and too signal-specific to support a
> general attenuation model on this dataset.

### 4.2 LiDAR geometric NLOS ≠ measured pseudorange NLOS
Joining LiDAR geometric-NLOS flags with DD truth (step8d):
- Agreement F1 = 0.270, recall = 0.171.
- 83% of LiDAR-flagged NLOS satellites show no measured pseudorange bias
  (|dd_resid| small).
- dd_resid vs ΔL: Spearman = −0.062 (no monotonic relationship).
- GPS partial correlation dd_resid ~ log(ρ_norm) | ΔL: r=−0.076, p≈0.000 — a
  faint incremental material association, but practically negligible.

> Takeaway: single-epoch geometric occlusion predicts *visibility blockage*, not
> *measured pseudorange error*. Most blocked rays still yield a usable, nearly
> unbiased pseudorange (diffraction, partial blockage, strong direct-path
> dominance, or reflection geometry that adds little excess path).

### 4.3 Apparent ML discrimination is satellite elevation, not LiDAR
Truth label |dd_resid|>20 m (745 positive) vs <5 m (3190 clean) (step8e/8f):

| Model | RF AUC | GBT AUC |
|-------|-------:|--------:|
| Elevation only (M1) | 0.908 | 0.957 |
| LiDAR only (M2) | 0.713 | 0.786 |
| Elevation + LiDAR (M3) | 0.907 | 0.952 |

Single-feature AUC: elevation 0.840; ρ_norm 0.557; ΔL 0.535; incidence 0.530.
RandomForest feature importance: elevation 0.746. The marginal contribution of
LiDAR features (M3 − M1) is −0.002 (RF) / −0.005 (GBT) — **zero within noise**.
A high headline AUC (0.906) is entirely satellite geometry, which is known and
free.

### 4.4 The residual LiDAR signal is cross-validation leakage
LiDAR features are *not* an elevation proxy (Spearman with elevation < 0.03;
elevation explains R² ≈ 0.003 of each). Residualizing elevation and testing the
pure LiDAR residual (step8g/8h):

| Cross-validation scheme | RF AUC | LogReg AUC | What it tests |
|---|---:|---:|---|
| (A) Random K-fold | 0.748 ± 0.014 | 0.597 ± 0.016 | leaky baseline |
| (B) GroupKFold by **satellite** | **0.533 ± 0.034** | 0.508 ± 0.081 | generalize to unseen satellites |
| (C) GroupKFold by time-block (60 s) | 0.648 ± 0.122 | 0.599 ± 0.109 | still leaks satellite identity |

- The only leakage-free generalization test, (B), gives 0.533 ≈ chance.
- The gap (A)−(B) ≈ 0.215 is the memorization a RandomForest extracts when a
  satellite's autocorrelated consecutive epochs are split across train and test.
- (C) at 0.648 is intermediate and unstable (±0.122) because the same satellite
  still appears in train and test at different time blocks; the (C)−(B) gap is
  exactly per-satellite-identity leakage, which does not transfer to a new route
  or day.
- 26 satellites, 283 sat×60 s blocks.

> Takeaway: the apparent residual LiDAR signal does not generalize. There is no
> satellite-independent LiDAR→NLOS mapping in single-epoch features on this
> dataset.

## 5. Discussion

### 5.1 Why single-epoch LiDAR fails here
- Geometric occlusion is necessary but far from sufficient for a biased
  pseudorange; receiver tracking often locks the direct/diffracted path.
- Reflectance encodes the *surface*, but the *excess path geometry* (which
  determines ΔL and hence the bias) is only weakly tied to surface material.
- High-reflectivity specular surfaces (glass/metal) can produce coherent
  reflections with *small* C/N0 drop, breaking the assumed monotonic
  material→attenuation relationship (consistent with the non-monotonic CN0_drop
  vs ρ pattern).
- Modulation matters: Galileo E1 BOC is largely NLOS-immune in C/N0, so pooling
  constellations dilutes any material effect.

### 5.2 A cross-validation leakage caution (methodological contribution)
GNSS measurements are strongly autocorrelated in time: a satellite tracked over
consecutive epochs yields near-duplicate feature rows and slowly varying errors.
Random K-fold places these near-duplicates on both sides of the split, letting
flexible models (RandomForest, boosting) memorize rather than learn. We show
this inflates AUC by ~0.2 (0.533 → 0.748). **Recommended practice:** report
GroupKFold by satellite (and/or by time block) for any GNSS NLOS classifier;
treat random-split AUC/F1 on epoch-level time series as optimistic. We note that
an earlier internal classifier reaching F1=0.985 was a victim of exactly this
plus label leakage. [TODO: position relative to literature that may report
random-split scores.]

### 5.3 Limitations
- A single 13-min route, one city, one receiver; results may differ on other
  geometries and with denser/longer LiDAR coverage.
- BeiDou subset IGSO-dominated; BeiDou MEO undersampled (n=30 severe) — its null
  is inconclusive.
- ρ_norm normalization uses simplified range/incidence compensation; sensor-level
  reflectance calibration could sharpen the material proxy.
- DD truth assumes the HKSC baseline reference is clean; residual reference-side
  multipath is possible.
- We test *single-epoch* features only; multi-epoch reflection-path / 2nd-order
  reflection modeling is explicitly out of scope and is the natural next study.

## 6. Conclusion

On a representative Hong Kong urban-canyon dataset, single-epoch LiDAR
reflectance and geometric features do **not** provide generalizable, independent
discrimination of GNSS pseudorange-domain NLOS errors beyond what satellite
elevation already supplies for free. A material→C/N0 attenuation effect exists
but only under severe GPS occlusion and is too weak to anchor a general model.
Apparent machine-learning success is attributable to satellite geometry and, in
the residual, to temporal-autocorrelation cross-validation leakage that vanishes
under satellite-grouped validation (AUC 0.748 → 0.533). We contribute a clean
DD-based evaluation protocol, a falsification-style diagnostic, and a concrete
caution against random cross-validation on autocorrelated GNSS time series.

---

## 附录 A：诊断脚本清单（可复现）

| 脚本 | 作用 | 关键结果 |
|------|------|---------|
| `lidar_step6e_verify_material.py` | 反射率↔CN0 死活验证（bootstrap/分层/偏相关） | GPS r=−0.058 CI 排除0；多星座 +0.006 |
| `lidar_step6f_multignss_rho_cn0.py` | 多星座 CN0_drop 数据构建 | 5438 匹配 G/C/E |
| `lidar_step6g_severe_per_constellation.py` | severe NLOS 逐星座交叉验证 | GPS severe r=−0.167（CI 排除0） |
| `lidar_step6h_beidou_orbit_check.py` | 北斗 GEO/IGSO/MEO 混淆检验 | IGSO 主导 86%，北斗结论不确定 |
| `lidar_step8c_psr_error_material.py` | 单接收机伪距误差（失败案例） | 城市峡谷钟差估计崩溃 |
| `lidar_step8d_dd_resid_material.py` | 双差残差 NLOS 真值 | F1=0.27；83% 假阳 |
| `lidar_step8e_lidar_discriminates_ddnlos.py` | LiDAR 判别 DD-NLOS 的 AUC | AUC 0.906（仰角主导） |
| `lidar_step8f_lidar_vs_elevation_ablation.py` | 仰角 vs LiDAR 消融 | 增量 ≈ 0 |
| `lidar_step8g_residualize_elevation.py` | 剥离仰角的残差判别力 | 随机CV 0.748（误导） |
| `lidar_step8h_grouped_cv_leakage.py` | 分组CV 泄漏检验 | 按卫星塌回 0.533 |

## 附录 B：待补充

- [ ] 图1：SPP vs GT 轨迹与误差分布（已有 HTML 报告，导出为矢量图）
- [ ] 图2：ρ_norm vs CN0_drop 分层散点（step6 系列）
- [ ] 图3：dd_resid vs ΔL 散点（step8d HTML）
- [ ] 图4：三种 CV 切分 AUC 对比柱状图（step8h，本文核心图）
- [ ] 表：LiDAR 传感器型号/频率/外参来源
- [ ] 相关工作引用补全（第2节）
- [ ] η_ref 与反射率标定细节（3.2 节）
