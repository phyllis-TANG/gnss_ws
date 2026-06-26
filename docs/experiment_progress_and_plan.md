# 实验进展与研究计划

**研究主题**：基于 LiDAR 反射率的卫星信号强度衰减模型建立  
**数据集**：UrbanNav-HK-Medium-Urban-1（香港，2021-05-17，约 13 分钟）  
**最后更新**：2026-06-10

---

## 一、已完成工作回顾

### 阶段 0：SPP 基准流水线

**目标**：跑通 GNSS 单点定位，建立误差基准。

**完成内容**：
- 搭建 ROS1 Noetic 容器环境，部署 del1RTK SPP 节点
- 编写 `rinex_to_rosbag.py`：将 RINEX obs+nav 转换为 ROS bag
- 修复关键 Bug（commit b2022f8）：RINEX3 历元时间误按 UTC 处理，导致 GPS TOW 多 18 秒，SPP 偏差约 16 km。修复后正常。
- 运行 SPP，与 UrbanNav 提供的 RTK/INS 地面真值对比

**结果**：
| 指标 | 数值 |
|------|------|
| 轨迹点数 | 635（SPP），787（GT），时间匹配 621 |
| 均值误差 | 84.8 m |
| RMS | 103.8 m |
| 95th 百分位 | 176.1 m |

符合香港城市峡谷 GNSS-only SPP 预期（50–200 m）。

---

### 阶段 1：LiDAR NLOS 检测

**目标**：利用 LiDAR 点云识别 GNSS 卫星是否为非视距（NLOS）。

**完成内容**：
- `lidar_step2_build_map.py`：从 Velodyne bag + NovAtel 轨迹构建 ENU 点云地图（urbannav_map.pcd，~1600 万点）
- `lidar_step3_multignss_azel.py`：提取每历元每颗卫星的方位角/仰角
- `lidar_step6_reflection_model.py`：光线投射检测 NLOS，输出 `lidar_reflection_model.csv`，含命中点坐标、路径长差 ΔL、入射角等

**结果**：
- GPS NLOS 率：40.8%（城市峡谷符合预期）
- 仅对 GPS 卫星（'G' 前缀）有标注，共 4140 条 NLOS 记录

---

### 阶段 2：多星座 SPP 探索

**目标**：尝试 GPS+BDS+Galileo 多星座 SPP，验证 NLOS 排除效果。

**完成内容**：
- `lidar_step7_multignss_spp.py`：多星座加权最小二乘 SPP + NLOS 排除/修正
- 分析 NLOS-排除可行性边界理论

**结果**：
| 方案 | 平均误差 |
|------|---------|
| GPS-only 基准 | 26.45 m |
| NLOS 排除 | 31.63 m（**更差**）|
| NLOS 修正 | 28.39 m |

**关键发现**：
- NLOS-排除可行性边界：`检测率 ≤ 1 - n_state/N_sats`，GPS-only 约为 30%
- 实际 NLOS 率 40.8% > 30% → 单历元硬排除在结构上不可行
- BDS/Galileo 无 NLOS 标注，排除 GPS NLOS 后 DOP 恶化（1.88→2.9），同时 BDS/GAL 偏差依然存在
- **结论**：保留多星座参与定位，不做粗暴排除

---

### 阶段 3：ML NLOS 分类器（初版）

**完成内容**：
- `lidar_step8_ml_nlos.py`：融合 LiDAR 几何特征 + GNSS 信号特征训练分类器

**结果（有数据泄漏）**：
| 指标 | 数值 |
|------|------|
| F1 | 0.985 |
| LiDAR 几何特征重要性 | 62.6% |
| CN0 特征重要性 | 4.4% |

**问题**：训练/测试集在同一历元内划分，同历元其他卫星特征泄露 → F1=0.985 不可信，需按时间重新划分。

---

### 阶段 4：LiDAR 强度地图重建

**背景**：原 `step2` 在读取 Velodyne 点云时丢弃了 intensity 字段，无法用于材质反射率研究。

**完成内容**：
- `lidar_step2b_build_map_intensity.py`：重建 ENU 点云地图，保留 intensity 字段
- 使用相同的 NovAtel 轨迹（ENU 坐标系一致）

**结果**：
- 输出 `urbannav_map_intensity.pcd`：8,343,143 点，4 列（x y z intensity），intensity 范围 0~252
- 耗时约 95 秒（33 GB bag，step=10 降采样）

---

### 阶段 5：NLOS 命中点材质反射率提取

**目标**：从 LiDAR 强度地图中提取每个 NLOS 反射命中点的材质反射率。

**物理模型**（Jutzi & Stilla 2006）：
```
I_measured = η × ρ × cos(α) / R²
ρ_norm = I × R² / (η_ref × cos(α))
其中 η_ref = median(I × R² / cos(α))，使 ρ_norm 中位数 = 1.0
```

**完成内容**：
- `lidar_step6b_extract_intensity.py`：KD-tree 最近邻查询（半径 0.8 m），计算 ρ_norm

**结果**：
| 指标 | 数值 |
|------|------|
| 命中率 | 84.7%（2331/2753）|
| mild NLOS ρ_norm 中位 | 3.66（附近光滑幕墙，高反射）|
| strong NLOS ρ_norm 中位 | 0.885 |
| severe NLOS ρ_norm 中位 | 0.844（远距粗糙面，低反射）|

---

### 阶段 6：ρ_norm 与 CN0_drop 相关性分析

**目标**：验证核心假设——材质反射率能否预测 GNSS 信号衰减量。

**方法**：
- 用 LOS 卫星拟合 `CN0_expected(elev)` 基准曲线
- `CN0_drop = CN0_expected - CN0_measured`
- 计算 ρ_norm 与 CN0_drop 的 Pearson/Spearman 相关系数

**完成内容**：
- `lidar_step6c_rho_vs_cn0.py`：生成 `rho_cn0_analysis.csv` + HTML 报告

**结果**：
| 子集 | n | Spearman r | p 值 |
|------|---|-----------|------|
| 全样本 | 2112 | -0.066 | 0.002 |
| mild | 335 | -0.082 | 0.135（不显著）|
| strong | 1430 | -0.009 | 0.733（不显著）|
| **severe** | 347 | **-0.195** | **0.0003（显著）**|

**关键发现——非单调的分位数模式**：
```
ρ [0.00, 0.42)  → CN0_drop = +2.17 dB
ρ [0.42, 0.84)  → CN0_drop = +3.10 dB  ← 最大衰减！
ρ [0.84, 1.45)  → CN0_drop = +2.45 dB
ρ [1.45, 3.07)  → CN0_drop = +0.83 dB  ← 高反射率，衰减最小
ρ [3.07, 85.0)  → CN0_drop = +1.54 dB
```

物理解释：
- **镜面体制（高 ρ，玻璃/金属）**：信号相干反射，CN0_drop 小，但伪距偏差大
- **漫反射体制（低 ρ，粗混凝土）**：能量散射，CN0_drop 大，信号弱

这一反直觉的双体制模式在已有文献中未见实验验证。

---

### 阶段 7：多变量 CN0_drop 预测模型 v1

**完成内容**：
- `lidar_step6d_cn0_model.py`（v1）：物理线性回归 + 随机森林

**结果（v1 问题）**：
| 模型 | 测试 R² | 备注 |
|------|---------|------|
| Baseline | 0.014 | 仅 severity 均值 |
| PhysLR | 0.038 | `log(R_hit)` 系数为负（✗，与理论矛盾）|
| RF | -0.18 | 严重过拟合（训练 0.61）|

RF 特征重要性中 elevation 占 56.1%，说明 CN0_expected 修正不彻底。

---

### 阶段 8：改进版 CN0_drop 预测模型 v2（当前）

**改进**：将 CN0_expected 从"全局 2 次多项式"改为"全局 sin(elev) 线性 + 逐颗卫星偏差校正"。

**物理依据**：
- 大气损耗 ∝ 1/sin(elev)，sin 模型比 elev 多项式更符合物理
- GPS 不同批次卫星（Block IIR/IIF/III）发射功率差约 1–3 dB
- 逐星偏差：G04 = −2.41 dB，G09 = +2.43 dB，范围达 4.84 dB

**结果对比**：

| 指标 | v1 | v2 |
|------|-----|-----|
| elevation vs CN0_drop 相关性 r | -0.135 | -0.099（改善 26%）|
| elevation RF 重要性 | 56.1% | **30.4%**（降幅近半）|
| ρ_norm RF 重要性 | 6.5% | **14.8%**（翻倍）|
| RF 训练/测试 R² | 0.61 / -0.18 | 0.37 / -0.003（过拟合改善）|
| `log(R_hit)` 系数 | -0.419 ✗ | **+0.202 ✓**（修正）|
| 全部物理系数符号 | 4/5 ✓ | **5/5 ✓** |

**v2 物理系数（全部符合理论）**：
```
log(ρ_norm):  -0.591  理论(-)  ✓  高反射率 → 衰减减小
log(R_hit):   +0.202  理论(+)  ✓  路径越远 → 衰减增大
-log(cos α):  +1.442  理论(+)  ✓  掠射角越大 → 衰减增大
sin(elev):    -1.496  理论(-)  ✓  高仰角 → 基准CN0更高
severity:     +2.411  理论(+)  ✓  mild < strong < severe
```

**为什么 R² 仍然只有 0.02**：
- CN0_drop 标准差 = 6.26 dB（噪声地板）
- 材质效应贡献约 1–2 dB（信号）
- 主要噪声：接收机 AGC 压缩、多径相位干涉（±3 dB 随机）
- 这是 u-blox F9P 在城市场景下的物理限制，不是模型错误

---

## 二、核心科学发现汇总

1. **物理模型得到验证**：改进 CN0_expected 后，Friis + LiDAR 方程的 5 个系数全部符号正确，支持物理假设。

2. **非单调的 ρ_norm vs CN0_drop 模式**：镜面高反射率 NLOS 的 CN0_drop 反而比漫反射低，揭示了镜面/漫反射双体制的物理机制。文献中未见类似实验验证。

3. **材质特征的真实贡献被仰角偏差掩盖**：逐星偏差校正后，ρ_norm 在 RF 中的重要性从 6.5% 升至 14.8%，证明原版分析低估了材质的作用。

4. **NLOS-排除可行性边界**：GPS-only 城市场景下，NLOS 率（40.8%）超过理论上限（30%），单历元硬排除在结构上不可行，这是 SPP 精度瓶颈的根本原因。

5. **LiDAR 与 ML 分类器的互补性**：LiDAR 几何特征贡献 62.6% 分类重要性，能检测镜面 NLOS（40.1%）；ML 独有特征能检测衍射型 NLOS（8.3%）（注：此结果含数据泄漏，需重新验证）。

---

## 二·补：决定性诊断线（step8c–8h，2026-06-26 更新）

> **重要**：本节结论**取代**上面"二、核心科学发现"中第 3/5 条的乐观表述。
> 经过一套逐层剥离混淆变量的诊断，最终结论是严谨的**阴性/边界结果**。

### 背景：从 CN0 目标转向伪距域 NLOS
CN0_drop 噪声（~6.4 dB）远大于材质信号（1–2 dB），且材质效应只在 GPS severe
子集勉强显著。于是改用更强的目标变量——伪距域 NLOS 误差。

### 诊断链条与结果

| 步骤 | 测试 | 结果 |
|------|------|------|
| step8c | 单接收机伪距误差 | ❌ 城市峡谷钟差估计崩溃（402/657 历元失败，5th=−60m 非物理） |
| step8d | 双差残差（HKSC 参考站）作干净 NLOS 真值 | dd_resid 5th=−9.1m（钟差解决）；**LiDAR几何NLOS vs DD实测 F1=0.27，83% 假阳**；dd~ΔL Spearman=−0.062 |
| step8e | LiDAR 特征判别 DD-NLOS 的 AUC | AUC=0.906，但**仰角单特征就 0.840，RF 重要性 74.6%** |
| step8f | 仰角 vs LiDAR 消融（M1/M2/M3） | **加 LiDAR 边际增量 ≈ 0**（M3−M1 = −0.002 RF / −0.005 GBT） |
| step8g | 剥离仰角后残差判别力 | LiDAR 不是仰角代理（相关<0.03）；残差随机CV RF=0.748（**误导**） |
| step8h | 分组CV 泄漏检验 | **按卫星分组 RF 塌回 0.533 ≈ 随机**；时间块 0.648±0.122（仍泄漏） |

### 最终科学结论
1. **单帧 LiDAR 反射率/几何特征，对伪距域 NLOS 无可泛化的独立判别力**——
   在卫星仰角（已知、免费）之外贡献约为零。
2. step8e 表面 AUC=0.906 全是**卫星几何（仰角）**；残差里的 0.748 是
   **时序自相关泄漏**，按卫星分组后塌回 0.533。
3. 材质→CN0 效应真实但仅限 GPS severe（r=−0.167，CI 排除0），过弱无法支撑通用模型。
4. **方法论贡献**：揭示 GNSS 时序数据上随机 CV 的泄漏陷阱（AUC 虚高 ~0.2）。
   旧 F1=0.985 分类器正是此陷阱 + 标签泄漏的受害者。

### step9a 多历元前提检验（2026-06-26 追加）
为决定是否值得做多历元/二次反射建模，做了星内去均值前提检验：
- **星内去均值**（控制卫星身份混淆）：dd↔ΔL=−0.069，dd↔log(ρ)=−0.075，**均 p<0.001**
- **时序平滑后**：dd↔ΔL=−0.118，dd↔log(ρ)=−0.101（平滑使相关翻倍 → 多历元有价值）
- **NLOS 持续性**：run 中位=2 历元，25% ≥5 历元，每星 NLOS 仅占 ~5% 时间
- **关键转折**：step8h 按卫星分组 AUC=0.533 让人以为是纯噪声；但星内分析证明
  **材质/几何与误差的关联真实存在**（p<0.001），只是效应极弱（R²≈1.4%）

### 最终定性（取代纯阴性）
结论从"纯阴性"升级为 **"真实但弱的正效应"**：
> LiDAR 反射率↔GNSS NLOS 误差的关联**真实存在**（星内 p<0.001，GPS severe 达 −0.167），
> 与材质衰减假设一致；但效应过弱（R²≈1.4%），单次反射特征不足以做定位改正。
> 提升到可用强度需**受控材质采集 + 传感器级反射率标定 + 多返回 LiDAR**，
> 而非更多同类车载数据（瓶颈是效应量不是样本量）。

### 方向决定（2026-06-26）
用户拍板：**写论文**，框架为"真实但弱的正效应 + 方法论(CV泄漏/仰角混淆)"。
草稿见 `docs/paper_draft_negative_result.md`（v0.2 已按此框架重写）。

---

## 三、待完成计划

> ⚠️ 下列"近期任务 A/B/C"是诊断前的旧计划，部分已被上节结论取代。
> 任务 A（修复时序泄漏）已由 step8h 完成并得出阴性结论；
> 任务 C 的乐观叙事已被 `paper_draft_negative_result.md` 的阴性叙事替换。
> 当前实际待办见本节末尾"★ 当前实际待办"。

### 近期（约 1–2 周）

**任务 A：修复 ML 分类器的时序泄漏**
- 当前问题：train/test 在同一历元内划分，泄漏 F1=0.985 不可信
- 修复方案：按 epoch 时间排序，前 70% 训练，后 30% 测试
- 对比实验：
  - 纯几何特征（无 ρ_norm）→ 基准
  - 加入 ρ_norm 后的 F1 提升 → 量化材质信息的边际贡献
- 输出脚本：`lidar_step8b_ml_nlos_temporal.py`

**任务 B：severe NLOS 子集深挖**
- 347 条 severe NLOS 中 r=-0.195，是目前最显著的相关子集
- 分析：是否特定建筑材质/方向导致？还是与 hit_dist、incidence 的联合效应？
- 方法：仅在 severe 子集上拟合多变量模型，看 R² 能否达到 0.05+

### 中期（约 1 个月）

**任务 C：整理成会议论文**

核心叙事（已有实验支撑）：
> "我们建立了基于 LiDAR 反射率的 GNSS 信号衰减物理模型。
> 实验发现：(1) ρ_norm 与 CN0_drop 呈非单调关系——镜面 NLOS 衰减反而更小，
> 揭示了镜面/漫反射双物理体制；(2) 逐星偏差校正后，物理模型的 5 个系数
> 全部符号正确，验证了 Friis+LiDAR 方程的适用性；(3) 材质特征在 RF 中的
> 重要性翻倍（6.5%→14.8%），证明 LiDAR 强度字段对 GNSS 信号质量有预测价值。"

候选投稿方向：ION GNSS+、IEEE/ION PLANS、IPIN（室内外导航）

**任务 D：文档整理**
- 完善 `research_roadmap.md`，加入 step6b/6c/6d 实验结果
- 更新 `step7_8_nlos_correction_results.md`，加入材质模型章节

### 长期（视数据和进展）

**任务 E：BDS/Galileo 的 NLOS 标注**
- 运行 `lidar_step3_multignss_azel.py` + step4 光线投射，为 BDS/GAL 生成 NLOS 标注
- 当前数据量：GPS 2112 条匹配样本；加入 BDS/GAL 预计可扩展 2–3 倍
- 完整 NLOS 标注后重做 multi-GNSS SPP，检验是否能改善定位精度

**任务 F：接入组内新数据集**
- 用新数据验证材质衰减模型的可迁移性（不同城市、不同接收机、不同卫星系统）
- 将 step2b、step6b、step6d 流水线迁移到新数据

**任务 G：CN0 加权 SPP**
- 用 step6d 预测的 CN0_drop 值作为伪距权重（`σ² ∝ CN0_drop`）
- 与现有仰角加权（`σ² ∝ 1/sin²(elev)`）对比
- 验证材质感知的信号质量模型能否改善 SPP 定位精度

---

## 四、当前文件和脚本清单

### 脚本（`/scripts/`）

| 脚本 | 功能 | 状态 |
|------|------|------|
| `rinex_to_rosbag.py` | RINEX obs+nav → ROS bag | ✅ 完成 |
| `save_trajectory.py` | 记录 SPP 轨迹 CSV | ✅ 完成 |
| `generate_analysis.py` | SPP 分析 HTML 报告 | ✅ 完成 |
| `lidar_step2_build_map.py` | 构建 ENU 点云（无 intensity）| ✅ 完成 |
| `lidar_step2b_build_map_intensity.py` | 构建 ENU 点云（保留 intensity）| ✅ 完成 |
| `lidar_step3_multignss_azel.py` | 提取卫星方位角/仰角 | ✅ 完成 |
| `lidar_step6_reflection_model.py` | 光线投射 NLOS 检测 | ✅ 完成 |
| `lidar_step6b_extract_intensity.py` | 提取 NLOS 命中点材质反射率 | ✅ 完成 |
| `lidar_step6c_rho_vs_cn0.py` | ρ_norm vs CN0_drop 相关性分析 | ✅ 完成 |
| `lidar_step6d_cn0_model.py` | 多变量 CN0_drop 预测模型 v2 | ✅ 完成 |
| `lidar_step7_multignss_spp.py` | 多星座 SPP + NLOS 排除/修正 | ✅ 完成 |
| `lidar_step8_ml_nlos.py` | ML NLOS 分类器（含泄漏）| ⚠️ 已证实为随机CV泄漏 |
| `lidar_step6e_verify_material.py` | 反射率↔CN0 死活验证 | ✅ GPS r=−0.058 CI排除0 |
| `lidar_step6f_multignss_rho_cn0.py` | 多星座 CN0_drop 数据构建 | ✅ 5438 匹配 |
| `lidar_step6g_severe_per_constellation.py` | severe 逐星座交叉验证 | ✅ 仅GPS robust |
| `lidar_step6h_beidou_orbit_check.py` | 北斗轨道混淆检验 | ✅ IGSO主导，不确定 |
| `lidar_step8c_psr_error_material.py` | 单接收机伪距误差 | ❌ 钟差崩溃（失败案例） |
| `lidar_step8d_dd_resid_material.py` | 双差残差 NLOS 真值 | ✅ F1=0.27 |
| `lidar_step8e_lidar_discriminates_ddnlos.py` | LiDAR 判别 DD-NLOS AUC | ✅ 0.906（仰角主导） |
| `lidar_step8f_lidar_vs_elevation_ablation.py` | 仰角 vs LiDAR 消融 | ✅ 增量≈0 |
| `lidar_step8g_residualize_elevation.py` | 剥离仰角残差判别力 | ✅ 随机CV 0.748（误导） |
| `lidar_step8h_grouped_cv_leakage.py` | 分组CV 泄漏检验 | ✅ 按卫星塌回 0.533 |

### 数据文件（容器 `/root/`）

| 文件 | 内容 | 大小 |
|------|------|------|
| `urbannav_map.pcd` | ENU 点云（xyz）| ~1600 万点 |
| `urbannav_map_intensity.pcd` | ENU 点云（xyz + intensity）| ~834 万点 |
| `lidar_reflection_model.csv` | NLOS 几何模型 | 2753 条 |
| `lidar_reflection_intensity.csv` | 加入 ρ_norm | 2753 条 |
| `rho_cn0_analysis.csv` | ρ_norm + CN0_drop 匹配 | 2112 条 |
| `cn0_model_results.csv` | 模型预测结果 | 2100 条 |

### 文档（`/docs/`）

| 文件 | 内容 |
|------|------|
| `step0_shenzhen_pipeline.md` | 早期深圳数据流水线记录 |
| `step1_spp_baseline.md` | SPP 基准结果文档 |
| `step7_8_nlos_correction_results.md` | NLOS 修正实验结果 |
| `research_roadmap.md` | 研究路线图（早期版本）|
| `experiment_progress_and_plan.md` | **本文档**：完整进展与计划 |
| `paper_draft_negative_result.md` | **论文草稿**：阴性/边界结果 + CV泄漏方法论 |

---

## 五、流水线运行顺序（从零开始）

```bash
# 0. 基础 SPP
python3 rinex_to_rosbag.py --obs *.obs --nav *.21n --out gnss.bag
# 启动 roscore + del1RTK + save_trajectory.py，播放 bag

# 1. 构建强度地图（约 95 秒）
python3 lidar_step2b_build_map_intensity.py \
  --bag /bags/*.bag --traj /root/novatel_trajectory.csv \
  --out /root/urbannav_map_intensity.pcd

# 2. 提取卫星方位角/仰角 + NLOS 光线投射
python3 lidar_step3_multignss_azel.py ...
python3 lidar_step6_reflection_model.py ...

# 3. 提取命中点材质反射率
python3 lidar_step6b_extract_intensity.py \
  --pcd /root/urbannav_map_intensity.pcd \
  --refl /root/lidar_reflection_model.csv \
  --out /root/lidar_reflection_intensity.csv

# 4. 相关性分析
python3 lidar_step6c_rho_vs_cn0.py \
  --refl /root/lidar_reflection_intensity.csv \
  --gnss /root/.../training_data.csv \
  --out_csv /root/rho_cn0_analysis.csv

# 5. 多变量预测模型
python3 lidar_step6d_cn0_model.py \
  --data /root/rho_cn0_analysis.csv \
  --gnss /root/.../training_data.csv \
  --out_csv /root/cn0_model_results.csv \
  --out_html /root/cn0_model_report.html
```
