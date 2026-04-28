# Step 0 — 深圳/东莞 自采数据 SPP 流程验证

**目标**：用自采的 u-blox 原始数据走通"UBX → RINEX → SPP"的完整链路，验证 GNSS 数据处理流程在云端环境可跑通。这是 UrbanNav 实验前的预热测试，不是城市峡谷误差评估。

> 与 Step 1 关键区别：本步使用 **RTKLIB `rnx2rtkp`**（命令行工具），**不是** del1RTK；接收机**静止**、**开阔天空**、**无地面真值对比**。

---

## 数据准备清单

- [x] **接收机**：u-blox F10T（多频多星座）
- [x] **原始数据**：`2026-4-15_174245_serial-COM5(1).ubx`（COM5 串口录制）
- [x] **采集时间**：2026-04-15 09:43–09:48 GPS（约 5 分 23 秒，323 历元）
- [x] **采集位置**：深圳/东莞交界，~22.5349°N, 113.9372°E, 海拔 ~170m（开阔天空，静止）

---

## 运行步骤

### 1. UBX → RINEX（用 RTKLIB 的 convbin）

```bash
convbin -r ubx -o spp_results/obs.obs -n spp_results/nav.nav \
        2026-4-15_174245_serial-COM5\(1\).ubx
```

输出：
- `obs.obs`（RINEX 3.04，混合星座 G/E/J/S，C1C+L1C 频点）
- `nav.nav`（GPS 星历）

### 2. RINEX → SPP（用 RTKLIB 的 rnx2rtkp）

```bash
rnx2rtkp -p 0 -m 15 -o spp_results/spp.pos \
         spp_results/obs.obs spp_results/nav.nav
```

参数说明：
- `-p 0`：定位模式 = Single（SPP）
- `-m 15`：最小卫星仰角 15°
- 输出 `spp.pos`：每个历元一行，含 lat/lon/h、Q（解类型）、ns（卫星数）、协方差

### 3. 生成轨迹图（Python 简单画散点）

```python
# 读 spp.pos 解析行：跳过 % 注释行，按列读 lat/lon
# 在 Cartesian/经纬度图上画散点 + 中心点
# 输出 spp_result.png
```

---

## 最终结果

| 指标 | 数值 |
|------|------|
| 有效历元 | 309 / 323 |
| 平均跟踪卫星数 | 5.6 颗 |
| 中心位置 | 22.5349°N, 113.9372°E |
| 中心海拔 | ~170 m |
| **2D RMS（东向）** | **56.5 m** |
| **2D RMS（北向）** | **40.4 m** |
| 解类型 | Q=5（Single / SPP） |

结论：开阔天空静止状态下 SPP 散布约 50m 量级，符合 u-blox 单频 SPP 在民用 GPS 下的典型水平。流程链路（UBX → RINEX → SPP → 可视化）全部跑通。

---

## 关键技术说明

| 项目 | 说明 |
|------|------|
| 处理工具 | **RTKLIB**（rnx2rtkp + convbin），不是 del1RTK |
| 处理方式 | 纯云端，无需 ROS 环境 |
| 时间系统 | obs 头部明确标记 `GPS`（无闰秒坑，因为 RTKLIB 内部正确处理）|
| 偏差度量 | 静止散布（无 GT），相对于解集中心的离散度 |
| 失败历元 | 14 / 323（卫星数不足或 PDOP 过大）|

---

## 与 UrbanNav Medium-Urban-1（Step 1）对比

| 维度 | Step 0（深圳静止） | Step 1（UrbanNav 动态）|
|------|------|------|
| 数据来源 | 自采 u-blox F10T | UrbanNav 公开数据集 |
| 场景 | 开阔天空，静止 | 城市峡谷，移动 |
| 处理工具 | RTKLIB rnx2rtkp | del1RTK（C++ 节点）|
| 是否有 GT | 否 | 有（RTK/INS）|
| 误差度量 | 散布 RMS（E56.5m / N40.4m）| 与 GT 偏差（均值 84.8m / RMS 103.8m）|
| 主要价值 | 验证流程链路 | 建立城市峡谷 SPP 基准线 |

---

## 本阶段的关键产物

- `spp_results/obs.obs` — RINEX 3.04 观测文件
- `spp_results/nav.nav` — GPS 星历
- `spp_results/spp.pos` — SPP 解算结果
- `spp_results/spp_result.png` — 轨迹散点图
- `spp_results/gnss_data.bag` — 后续转出的 ROS bag（commit 6f47975）

---

## 衍生工具（后续被 Step 1 沿用/替换）

| 脚本 | 作用 | 状态 |
|------|------|------|
| `scripts/ubx_to_rosbag.py` | UBX → ROS bag（用 pyubx2 + rosbags）| 已弃用，被 `rinex_to_rosbag.py` 替代 |
| `scripts/make_map.py` | 早期独立画图工具 | 被 `generate_analysis.py` 整合 |
| `scripts/HOW_TO_RUN.md` | 浏览器→终端粘贴脚本的速查表 | 仍适用 |

---

## 经验教训（用到 Step 1）

1. **`pyubx2` 单位坑**：返回的 lat/lon 已经是十进制度数，不是原始整数（commit b2e60cc 修复）
2. **`rosbags` 库 CDR 格式不兼容 ROS1**：必须改用原生 `import rosbag`（Step 1 已改）
3. **流式写入 CSV**：每个历元立刻 flush 到磁盘，Ctrl+C 不丢数据（沿用到 `save_trajectory.py`）
