# 研究方案 — 材料感知的 LiDAR 辅助 GNSS NLOS 改正

> "LiDAR 反射率 → GNSS 多径"研究方向 · 起草于现有 UrbanNav LiDAR-NLOS pipeline 之上
> 状态:初稿,待与导师 align 后细化

---

## 1. 背景 · 当前流水线为什么不够

我们已经跑通的端到端流程是:**LiDAR 3D 点云 → 体素占用地图 → 沿卫星方向射线追踪 → 判 LOS/NLOS → 单反 ΔL 几何改正 → 多星座 SPP**。在 UrbanNav Medium-Urban-1 上得到:NLOS 比例 59.5%,baseline 26.45m,**correction 模式 39.97m(反而比 baseline 差 51%)**。

诊断已经定位到核心短板:**我们整个流水线只用了 LiDAR 的几何信息(点在哪),完全没有用到 intensity / reflectivity 通道**。这导致:

- 反射面用 PCA 估法向量,**41% 的点平面性 < 0.5**,法向量不可信 → ΔL 偏差 5–15m;
- 改正"一刀切":不管反射面是玻璃幕墙、混凝土、还是树叶,都一律按 `ΔL = 2·d·cos θ` 公式硬减——但这个公式只在**镜面反射**下成立;
- 没有任何机制告诉算法"这条 NLOS 路径的反射可信度有多高"。

LiDAR 每个点本身就带 **回波强度(intensity)** 这一通道,**它编码的正是表面材料信息**——这是我们手上现有、但被完全丢弃的关键信号。

## 2. 核心研究假设

> **LiDAR intensity + 几何 联合,能比纯几何更准确地预测 GNSS NLOS 的多径延迟 ΔL 与其置信度。**

物理依据:
- **LiDAR intensity 编码材料反照率** —— 玻璃高且不稳、混凝土中等稳定、植被低且杂乱、金属饱和;
- **GNSS L1 信号反射遵循 Fresnel 定律**,反射系数 Γ 由材料相对介电常数 ε_r 决定(玻璃 ε_r ≈ 5–7,Γ ≈ 0.3–0.5;混凝土 ≈ 4–6;植被无 Γ 概念,以散射/吸收为主);
- 因此 **LiDAR 强度 → 材料类别 → ε_r → Fresnel Γ(θ) → GNSS 多径功率/ΔL 不确定度** 这条链路是物理上说得通的。

⚠️ **必须区分**:LiDAR(905 nm)和 GNSS(λ = 19 cm)波长差 5 个数量级,**对同一面墙"看到"的粗糙度完全不同**(毛玻璃 LiDAR 看是漫反射,GNSS 看近镜面)。所以 **LiDAR 反射率不能直接当 GNSS 反射率用,只能作为"材料先验"**,再查 GNSS 频段的 Fresnel 表。这是方案物理严谨性的关键。

## 3. 文献定位 · gap 在哪

| 已有工作 | 用了什么 | 没做什么 |
|----------|----------|----------|
| Höfle & Pfeifer (2007), ISPRS | LiDAR intensity 标定方法学 | 不涉及 GNSS |
| Wen & Hsu (2019/2021), NAVIGATION + arXiv | LiDAR **几何** → GNSS NLOS / RTK | **未用 intensity 通道** |
| Adjrad & Groves, *J. Navigation* | C/N0 + 3D 建筑模型 shadow matching | 几何源是 OSM,非 LiDAR;材料不区分 |
| 5G mmWave NLOS 文献 (e.g. 室内多径定位) | 点云 + 材料分类辅助多径建模 | 频段不同,方法可借鉴但不能直接搬 |
| 经典电磁(Fresnel) | 材料 ε_r → Γ 完整理论 | 没人系统性应用到城市 GNSS SPP 改正 |

**Gap 一句话**:**"LiDAR intensity → 材料 → Fresnel → GNSS NLOS 改正"这条全链路目前在 GNSS 城区定位文献中没有系统性工作**。这正是本方案的切入点。

## 4. 技术路线(4 步)

```
[LiDAR 点云 + intensity]
         │
   ① intensity 标定与归一化         ← 距离、入射角、传感器型号修正
         ↓
   ② 材料分类(3 类)                 ← 玻璃-金属 / 混凝土-砖 / 植被-杂
         ↓
   ③ 材料 → Fresnel Γ(θ) 映射        ← 查表 ε_r,按入射角算 Γ
         ↓
   ④ 集成进当前 Step 4/6/7 改正       ← 用 Γ 做 ΔL 置信度加权 + 强散射 FDE
```

**关键设计选择**:第一版**不追求精确反射功率**,而是按材料把 NLOS 分成三类置信度等级——

| 反射面材料(由 LiDAR intensity 推断) | GNSS 反射性质 | 改正策略 |
|------------------------------|----------------|-----------|
| 玻璃 / 金属(强 + 不稳) | 强镜面,ΔL 公式有效 | **采用 ΔL 改正,正常权重** |
| 混凝土 / 砖(中 + 稳) | 弱镜面 + 部分吸收 | **采用 ΔL,但降权 0.5** |
| 植被 / 杂(弱 + 散乱) | 无相干反射,信号≈噪声 | **不做 ΔL 改正,直接 exclude 或大幅降权** |

这就把第二阶段的 Fresnel 精细建模留作扩展,既能拿到第一版结果,又给论文留出可深化的层次。

## 5. 与现有 pipeline 的兼容性

**不重建任何已有代码**,仅做四处增量:
- `lidar_step2_build_map.py`:读 PCD 时把 `intensity` 通道一起存到体素地图;
- `lidar_step4_ray_casting.py`:射线撞击点处读取该体素的平均 intensity;
- `lidar_step6_reflection_model.py`:新增材料分类 + Γ 计算,与现有 PCA 法向量并列作为"软证据";
- `lidar_step7_multignss_spp.py`:WLS 权重里加入材料置信度因子(目前权重只跟仰角/C/N0 相关)。

现有的 11,050 条 NLOS、Step 8 残差验证流程**全部可复用**——也就是说,第一版结果可以**直接与现有 baseline / correction 数字对比**。

## 6. 实验设计

| 项目 | 设定 |
|------|------|
| 数据集 | UrbanNav Medium-Urban-1(已跑通)+ Hard-Urban(需新跑)|
| 评估指标 | (a) 材料分类 precision/recall(弱监督);(b) ΔL 估计 RMSE(对 Step 8 GT-反算残差);(c) SPP 水平定位 mean / RMS / 95th |
| 消融对照 | baseline (现有几何) → +intensity 材料分类 → +Fresnel 加权 → full |
| 关键对比 | 与 Wen et al. 2019(同场景纯几何 LiDAR-NLOS 在 HK 上 +47%)对齐评测 |

材料分类的 ground truth 用 **OSM building tags + 人工抽样标注 ~200 个点** 做弱监督,不依赖大规模标注。

## 7. 12 周里程碑

| 周次 | 任务 | 产出 |
|------|------|------|
| W1–2 | LiDAR intensity 读取 + 距离/入射角归一化 | 改造后的 Step 2 / Step 4,带 intensity 列的 CSV |
| W3–4 | 三分类材料标签(KMeans 初版 + OSM 弱监督校正) | 材料地图可视化 + 分类精度报告 |
| W5–6 | Fresnel Γ 查表 + 集成进 Step 6/7 | 第一版"材料感知改正"SPP 结果 |
| W7–8 | 消融实验 + 与 baseline 完整对照 | 实验表 + 误差分析报告 |
| W9–10 | UrbanNav Hard-Urban 复跑,泛化验证 | 第二数据集结果 |
| W11–12 | 论文初稿(目标 ION GNSS+ 或 IEEE IV) | 8–10 页 conference paper |

## 8. 主要风险与缓解

| 风险 | 缓解方案 |
|------|----------|
| LiDAR intensity 跨传感器/场景标定难 | 用**局部相对值**(同一射线邻域内 z-score),避免依赖绝对标定 |
| 材料分类无大规模 GT | OSM building tags + ≤200 点人工标注的弱监督;初版只求三分类够用 |
| LiDAR 与 GNSS 波长差异引入材料先验偏差 | 在论文中明确"用作材料先验,不作 GNSS 反射系数本身";第一版避免对单点功率做强声明 |
| Fresnel 公式假设理想平面 | 当 PCA planarity < 阈值时,降级为"散射"类别,不强用 Γ |
| 改进后仍打不过 baseline | 至少能解释清楚:correction 负收益的根因是材料盲区,这本身就是有价值的负结果论文素材 |

## 9. 与现有"硬伤清单"的对应

之前在成果分析里列过 5 个"性价比最高的改进":本方案直接覆盖其中三项 ——
- ✅ **PCA 法向量置信度加权** → 材料分类做置信度;
- ✅ **改正不一刀切** → 三档差异化策略;
- ✅ **跑第二个数据集** → W9–10 里程碑;
- 另两项(加 C/N0 软先验、加 FDE/RAIM)与本方案正交,可并行推进。

---

## 参考文献(初步)

1. Wen, W., Hsu, L.-T. (2019). Correcting NLOS by 3D LiDAR and building height to improve GNSS SPP. *NAVIGATION* 66(4). doi:10.1002/navi.335
2. Wen, W., Hsu, L.-T. (2021). 3D LiDAR Aided GNSS NLOS Mitigation in Urban Canyons. arXiv:2112.06108
3. Höfle, B., Pfeifer, N. (2007). Correction of laser scanning intensity data. *ISPRS J. Photogrammetry & Remote Sensing*, 62(6).
4. Lau, L., Cross, P. (2007). Development and testing of a new ray-tracing approach to GNSS carrier-phase multipath modelling. *J. Geodesy* 81(11). doi:10.1007/s00190-007-0139-z
5. Adjrad, M., Groves, P. D. Integration of Shadow Matching with 3DMA Ranging. *J. Navigation*.
6. Groves, P. D. (2011). Shadow Matching: A New GNSS Positioning Technique for Urban Canyons. *J. Navigation*.
7. UrbanNav (2023). Open-Source Multisensory Dataset. *NAVIGATION* 70(4). doi:10.33012/navi.602

---

**一句话总结**:**本方案是把当前流水线从"只用 LiDAR 几何"升级为"用 LiDAR 几何 + 材料(intensity)",物理上有 Fresnel 定律支撑,工程上只需在现有四个脚本上做增量改造,预期能把 correction 模式从 -51% 翻成正收益,同时形成会议论文级贡献。**
