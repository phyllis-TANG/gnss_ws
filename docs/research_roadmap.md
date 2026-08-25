# GNSS NLOS Research — 研究路线图与文献综述

**数据集**: UrbanNav HK Medium-Urban-1 (2021-05-17, 香港)  
**研究阶段**: 探索期 → 会议论文 → 毕业论文  
**最后更新**: 2026-06-09

---

## 一、研究主题

> 研究全球导航卫星系统信号反射的几何学（多路径效应和非视距效应）
> **基于LiDAR反射率的卫星信号强度衰减模型建立**

核心思路：LiDAR点云不仅提供几何信息（距离、法向量），还提供**材质反射率**（intensity字段）。
不同材质对GNSS L1信号的反射能力不同（玻璃幕墙反射系数0.5-0.7，混凝土更低），
从而导致NLOS路径上信号的CN0衰减程度不同。

---

## 二、已完成的研究历程

### 阶段0：环境搭建与数据预处理
- ROS1容器（ros1_gnss）+ del1RTK + pyrtklib
- RINEX obs/nav → rosbag 转换脚本（`rinex_to_rosbag.py`）
- **解决闰秒Bug**：GPS时间与UTC混淆导致卫星位置偏差~70km，修复后SPP恢复正常

### 阶段1：SPP基线建立
- GPS-only WLS SPP: **26.45 m**（del1RTK, 3D）/ **11.74 m**（pyrtklib, 2D horizontal）
- GT对比：UrbanNav SPAN-CPT RTK/INS，787点匹配657点
- 结论：符合香港城区GNSS-only SPP预期（50–200 m文献范围）

### 阶段2：LiDAR建图（FAST-LIO2）
- Velodyne LiDAR + IMU → FAST-LIO2 SLAM
- 输出：`scans.pcd`（143M点，含 **x y z intensity normal_x normal_y normal_z curvature**）
- 输出：`fastlio_colored.ply`（18.25M点，xyz + 法向量 + 相机RGB）
- 2-bounce射线追踪 → `lidar_reflection_model.csv`（每颗卫星×每历元的反射几何）

### 阶段3：基于几何特征的NLOS检测与SPP修正实验

| 方法 | 结果 | 结论 |
|------|------|------|
| DD残差修正 (t=5m) | **-0.88 m (-3.3%)** | ✅ 唯一有效的几何修正 |
| 卫星排除 (DD阈值) | 变差 | ✗ 损失DOP |
| CN0降权 | +0.66 m | ✗ false positive > benefit |
| FDE/RAIM | **+9 m** | ✗ 66%NLOS违反单故障假设 |
| Planarity-ΔL修正 | +0.24 m | ✗ 法向量质量不足 |

**FDE/RAIM失效的根本原因**：经典RAIM假设故障是少数，城区66.5% NLOS完全违反该假设。
WLS解被多数NLOS拉偏 → 后验残差中LOS卫星反而看起来最"异常" → 排除LOS → DOP崩溃。

### 阶段4：ML信号分类器（del2AINLOS RF）
- 特征：{CN0, 仰角, DD残差}，训练集：smallRoundUrbanV2x
- 5折CV: Acc=92.9%, F1=0.805
- **RF排除: -4.18 m (-35.7%)** ← 迄今最佳结果
- RF降权: -0.11 m（软权重依然无效，与阶段3一致）

### 阶段5：LiDAR几何 vs ML信号 互补性分析

3091条对齐记录（epoch × sat_id）：

| 类别 | 数量 | 物理含义 |
|------|------|---------|
| Both LOS | 592 (19.2%) | 真实LOS，两者一致 |
| **LiDAR-only NLOS** | 1238 (40.1%) | 镜面/小时延NLOS：几何上有2-bounce，但CN0≈正常，DD残差小 |
| **ML-only NLOS** | 256 (8.3%) | 衍射/散射NLOS：有信号劣化，但无干净2-bounce反射面 |
| Both NLOS | 1005 (32.5%) | 两者都能检测到的NLOS |

Jaccard = 0.402：两个方法是**独立的互补探测器**，各自捕捉不同物理机制的NLOS。

### 阶段6：融合分类器（信号 + LiDAR几何）
- 特征：{CN0, 仰角, DD残差} + {lidar_hit, planarity, delta_L_m, hit_dist_m, incidence_deg}
- 融合GT：nlos_label OR lidar_hit → 73.7% NLOS率
- Fusion RF: **F1 = 0.985**（含数据泄漏；无泄漏参考版F1=0.954）
- **LiDAR几何特征占总重要性62.6%；CN0仅占4.4%**

### 阶段7：SPP验证与关键发现

**NLOS-Exclusion Feasibility Boundary（关键发现）**

> 在GPS-only深度城区（N_sats ≈ 5.7颗/epoch），NLOS检测率与卫星稀缺性具有同一物理根源
>（高楼同时导致：①部分卫星被遮挡→星数少 ②剩余低仰角卫星穿过建筑间隙→NLOS）。
> 因此，最需要排除NLOS的历元，正是卫星总数最少的历元（通常仅4颗），
> 排除1颗后n_used=3 < n_state=4，WLS秩不足→全部回退基线，改善量=0%。

可行性边界公式：
```
detection_rate ≤ 1 - (n_state / N_sats) = 1 - 4/5.7 ≈ 30%
```
然而真实NLOS率 = 40.8% > 30%，GPS-only单历元排除在结构上无法覆盖全部NLOS。

| 场景 | 检测率 | 均值卫星数 | 排除后剩余 | 可行? |
|------|--------|-----------|-----------|------|
| del2AINLOS RF (保守7.2%) | 7.2% → 0.4颗/ep | 5.7 | 5.3 | ✅ 勉强可行 → -35.7% |
| 融合模型 (73.7%) | 73.7% → 4.1颗/ep | 5.6 | 1.5 | ✗ 完全不可行 |
| GPS+BDS多星座 (假设) | 73.7% → ~11颗/ep | ~15 | ~4 | ✅ 可行，预期大改善 |

---

## 三、在研究领域中的位置

### 相关工作对比

| 工作 | 方法 | LiDAR用途 | 是否用intensity |
|------|------|-----------|---------------|
| Wen & Hsu 2019/2022 | 3D地图+射线追踪 → NLOS排除/修正 | 几何检测 | ❌ |
| Miura et al. 2012 (*Sensors*) | LiDAR数字表面模型 → 多径幅度/延迟估计 | 几何建模 | ❌ |
| del2AINLOS (本研究) | DD残差+RF分类 → NLOS排除 | 不用LiDAR | — |
| Parvizi et al. 2017 (*ION*) | GNSS散射功率 + LiDAR联合测量 | 反射功率测量 | ✅ 但用于**海面**遥感 |
| GNSSNLOSDetector (GitHub) | ML多星座检测 | 不用LiDAR | — |
| **本研究目标** | **LiDAR intensity(材质反射率) → CN0衰减预测** | **几何+材质** | **✅ 城区建筑** |

**文献空白**：目前没有工作将城区建筑表面的LiDAR intensity（材质反射率）与GNSS L1信号CN0衰减定量关联。这是本研究的核心创新点。

### 关键参考文献

1. **Wen & Hsu 2019** - 3D LiDAR Aided GNSS NLOS Mitigation, *NAVIGATION* — 最近的几何NLOS方法，-47%，使用测量3D地图
2. **Miura et al. 2012** - Multipath Estimation from Joint GNSS+LiDAR, *Sensors* 12(11) — LiDAR DSM+GNSS多径估计，只用几何
3. **Parvizi et al. 2017** - Coordination of GNSS Signals with LiDAR for Reflectometry, *ION* — GNSS反射功率+LiDAR，海面
4. **Li et al. 2015** - NLOS Correction/Exclusion via RAIM+City Models, *Sensors* — DOP恶化问题文献证据
5. **ISPRS 2009** - LiDAR Intensity Normalization by Range and Incidence Angle — intensity归一化公式
6. **Zhang et al. 2021** - GNSS Diffraction Models in Urban Areas, *NAVIGATION* 68(2) — 衍射模型，CN0仿真

---

## 四、核心物理模型

### LiDAR强度方程（激光雷达方程简化）
```
I_measured = η × ρ × cos(α) / R²
```
- `η`：系统常数（传感器标定，可相对归一化）
- `ρ`：**表面材质反射率**（目标量）
- `α`：激光入射角（= `incidence_deg`，已有）
- `R`：距离（= `hit_dist_m`，已有）
- `I_measured`：LiDAR强度（来自 `scans.pcd` intensity字段，待提取）

**反解材质反射率**：
```
ρ_norm = I_measured × R² / (η_ref × cos(α))
```

### GNSS信号衰减模型（待建立）
```
CN0_NLOS = CN0_free_space(elevation) - L_path(Δpath) - L_material(ρ_norm, α)
```
- `L_path(Δpath)`：额外路径损耗 ≈ 20·log10(1 + Δpath/λ)（自由空间近似）
- `L_material(ρ_norm, α)`：材质散射损耗，是本研究要量化的项

### 核心假设（待验证）
- **高反射率面**（玻璃幕墙，ρ高）→ NLOS信号能量损失少 → CN0下降小 → 信号"更强但有偏差"
- **低反射率面**（粗糙混凝土，ρ低）→ NLOS信号能量损失大 → CN0明显下降 → 更容易被ML检测到

---

## 五、研究路线图

### 近期（现有数据，可立即推进）

**Step A：坐标系对齐（技术前提）**
- FAST-LIO2地图帧 → ENU坐标系的变换矩阵
- 需要找FAST-LIO2输出的轨迹文件（.tum或poses.csv）
- 或通过地面控制点手动配准

**Step B：intensity提取与相关性分析**
- 对 `lidar_reflection_model.csv` 每行hit点 → KD-tree查 `scans.pcd` → 获取intensity
- 计算 `ρ_norm = I × R² / cos(α)`
- 绘制散点图：`ρ_norm` vs `CN0_drop`（相对仰角模型的偏差）
- 统计：Pearson/Spearman相关系数，分材质类（severity = strong/severe/mild）

**Step C：初步衰减回归模型**
- 输入：`delta_L_m`, `ρ_norm`, `incidence_deg`, `hit_dist_m`
- 输出：预测 `CN0_drop`
- 方法：线性回归（可解释性强，适合首篇）→ 再试RF/NN

**Step D：多星座SPP验证（可并行）**
- 数据已有：`hksc137c.21m`（混合星座nav）+ `ublox.m8t.GC.obs`（GPS+BDS）
- 验证NLOS-Exclusion Feasibility Boundary：多星座能否突破GPS-only的scarcity限制

### 中期（会议论文方向）

**方向1：LiDAR反射率 → GNSS信号质量预测**（ION GNSS+ / ISPRS / IEEE GRSL）
- 核心贡献：首个用LiDAR intensity归一化材质反射率预测城区GNSS CN0衰减的工作
- 验证：UrbanNav Medium-Urban-1 + 一个新数据集

**方向2：NLOS-Exclusion可行性边界分析**（ION ITM / ENC）
- 核心贡献：GPS-only城区硬排除的结构性限制量化（`detection_rate ≤ 1 - n_state/N_sats`）
- 延伸：多星座下边界移动→排除可行性提升

### 长期（毕业论文框架）

```
输入层
  ├── GNSS原始观测量（伪距、多普勒、CN0）
  └── LiDAR点云（几何 + intensity）

特征提取层
  ├── 信号特征：CN0_drop, DD残差, 仰角
  ├── 几何特征：delta_L_m, incidence_deg, hit_dist_m
  └── 材质特征：ρ_norm（LiDAR intensity归一化）

模型层
  ├── NLOS检测：融合三类特征的分类器
  └── 信号衰减：CN0预测模型（物理引导的回归）

应用层
  ├── 改进NLOS检测权重
  ├── 伪距修正（ΔL × f(ρ, α)）
  └── 多星座WLS/FGO SPP定位

验证层
  ├── UrbanNav Medium-Urban-1（本研究基准）
  ├── UrbanNav其他序列
  └── 组内新收数据
```

---

## 六、当前待解决的技术问题

| 问题 | 状态 | 解决方向 |
|------|------|---------|
| scans.pcd坐标系 vs reflection model ENU不对齐 | ⏳ 待解决 | 找FAST-LIO2轨迹文件；或ICP配准 |
| lidar_reflection_model.csv无intensity列 | ⏳ 待提取 | 对齐后KD-tree查询scans.pcd |
| GPS-only多星座扩展 | ⏳ 待实验 | hksc137c.21m已有，m8t.GC.obs已有 |
| 材质分类标签 | ⏳ 待建立 | 结合ρ_norm + 相机RGB（fastlio_colored.ply）|
| 第二数据集验证 | ⏳ 未来 | UrbanNav其他序列或组内新数据 |

---

## 七、数据资产清单

| 文件 | 位置（容器内） | 内容 |
|------|--------------|------|
| `UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs` | `/root/urbannav_gnss/` | 主GNSS obs（GPS+多星座） |
| `hksc137c.21n` | `/root/urbannav_gnss/` | GPS nav（已用） |
| `hksc137c.21m` | `/root/urbannav_gnss/` | 混合星座nav（待用） |
| `hksc137c.21o` | `/root/urbannav_gnss/` | HKSC基站obs（待用于多星座DD）|
| `scans.pcd` | `/root/gnss_ws/src/.../fast_lio_multi/PCD/` | 143M点，**含intensity** |
| `fastlio_colored.ply` | `/root/` | 18.25M点，法向量+相机RGB |
| `lidar_reflection_model.csv` | `/root/` | 2243条NLOS几何记录（无intensity）|
| `training_data.csv` | `/root/gnss_ws/src/.../del2AINLOS/data/UrbanNavMedium/` | 3726条信号特征+DD标签 |
| `nlos_labels_clean.csv` | `del2AINLOS/results/UrbanNavMedium/` | per-sat NLOS标签 |
| `fusion_nlos_rf.pkl` | `/root/` | 融合分类器模型 |
