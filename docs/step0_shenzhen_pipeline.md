# Step 0 — 深圳/东莞 自采数据 SPP 流程验证

**目标**：用自采的 u-blox 原始数据走通完整链路，**两条路径都验证成功**：
- **路径 A（云端 RTKLIB）**：UBX → RINEX → SPP（rnx2rtkp）
- **路径 B（ROS del1RTK）**：UBX → ROS bag → del1RTK SPP

这是 UrbanNav 实验前的预热测试：开阔天空、静止接收机、5 分钟数据。

---

## 数据准备清单

- [x] **接收机**：u-blox F10T（多频多星座）
- [x] **原始数据**：`2026-4-15_174245_serial-COM5(1).ubx`（COM5 串口录制）
- [x] **采集时间**：2026-04-15 09:43–09:48 GPS（约 5 分 23 秒，323 历元）
- [x] **采集位置**：深圳/东莞交界，~22.5349°N, 113.9372°E, 海拔 ~170m（开阔天空，静止）

---

## 路径 A：云端 RTKLIB（无需 ROS）

### A1. UBX → RINEX

```bash
convbin -r ubx -o spp_results/obs.obs -n spp_results/nav.nav \
        2026-4-15_174245_serial-COM5\(1\).ubx
```

输出 `obs.obs`（RINEX 3.04，混合星座 G/E/J/S）+ `nav.nav`。

### A2. RINEX → SPP

```bash
rnx2rtkp -p 0 -m 15 -o spp_results/spp.pos \
         spp_results/obs.obs spp_results/nav.nav
```

参数：`-p 0` SPP 模式，`-m 15` 仰角掩码 15°。

### A3. 结果

| 指标 | 数值 |
|------|------|
| 有效历元 | 309 / 323 |
| 平均跟踪卫星数 | 5.6 |
| 中心位置 | 22.5349°N, 113.9372°E, ~170m |
| 2D RMS（东向）| 56.5 m |
| 2D RMS（北向）| 40.4 m |
| 解类型 | Q=5（Single） |

---

## 路径 B：ROS del1RTK（容器内）

### B1. UBX → ROS bag

```bash
python3 /root/ubx_to_rosbag.py \
  --ubx /root/ubx_data/input.ubx \
  --out /root/gnss_ws/data/bags/gnss_data.bag
```

发布 topics：
- `/ublox_driver/range_meas` (GnssMeasMsg)
- `/ublox_driver/ephem` (GnssEphemMsg)
- `/ublox_driver/receiver_lla` (NavSatFix)

### B2. 运行 SPP（3 个终端）

**终端 1 — roscore**
```bash
source /root/gnss_ws/devel/setup.bash
roscore
```

**终端 2 — 记录轨迹**
```bash
source /root/gnss_ws/devel/setup.bash
python3 /root/save_trajectory.py
```

**终端 3 — 跑 SPP**
```bash
source /root/gnss_ws/devel/setup.bash
roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_ws/data/bags/gnss_data.bag \
  bag_rate:=1 \
  exclude_glonass:=true \
  rviz:=false
```

> 注意：深圳数据用 `exclude_glonass:=true`（u-blox F10T 录制未含 GLONASS 星历），UrbanNav 用 `false`。

### B3. 成功标志

终端 3 输出：
```
[SPP] Epoch solved: lat=22.5349  lon=113.9372  alt=170.5m
```

bag 播完后回终端 2 按 Ctrl+C，CSV 写入 `/root/gnss_ws/spp_results/spp_trajectory.csv`，并生成 `trajectory.html` 离线地图。

### B4. 结果

del1RTK 的解算结果与 RTKLIB 一致：中心 22.5349°N, 113.9372°E, ~170m。
两条路径**互相验证**，确认 ROS pipeline（UBX→bag→del1RTK SPP）链路正确。

---

## 完整产物清单（仓库 `spp_results/` 目录）

- `obs.obs` — RINEX 3.04 观测文件（路径 A 输入 / B 参考）
- `nav.nav` — GPS 星历
- `spp.pos` — RTKLIB 解算结果（路径 A）
- `spp_result.png` — 路径 A 散点图
- `gnss_data.bag` — UBX 转出的 ROS bag（路径 B 输入）

---

## 关键技术说明

| 项目 | 说明 |
|------|------|
| 时间系统 | obs 头部明确标记 `GPS`（无闰秒坑：RTKLIB 内部正确处理；ROS 路径在转 bag 时也正确）|
| GLONASS 处理 | 深圳数据没有 GLONASS 星历，必须 `exclude_glonass:=true` |
| 接收机状态 | 静止 → 评估指标是"散布"而非"轨迹误差"|
| 验证方法 | 两条独立路径解算同一份原始数据，结果一致即视为链路通畅 |

---

## 与 UrbanNav Medium-Urban-1（Step 1）对比

| 维度 | Step 0（深圳静止） | Step 1（UrbanNav 动态）|
|------|------|------|
| 数据来源 | 自采 u-blox F10T UBX | UrbanNav 公开 RINEX 3 obs/nav |
| 场景 | 开阔天空，静止 | 城市峡谷，移动 |
| 输入路径 | UBX → bag（`ubx_to_rosbag.py`）| RINEX → bag（`rinex_to_rosbag.py`）|
| del1RTK 配置 | `exclude_glonass:=true` | `exclude_glonass:=false` |
| 是否有 GT | 否 | 有（RTK/INS）|
| 误差度量 | 散布 RMS（E56.5m / N40.4m）| 与 GT 偏差（均值 84.8m / RMS 103.8m）|
| 主要价值 | 验证两条 SPP 路径链路通畅 | 建立城市峡谷 SPP 基准线 |

---

## 本阶段沉淀的工具（用到 Step 1）

| 脚本 | 作用 | 后续演进 |
|------|------|------|
| `scripts/ubx_to_rosbag.py` | UBX → ROS bag（pyubx2 + rosbags 库）| Step 1 改用 `rinex_to_rosbag.py`（输入是 RINEX）|
| `scripts/save_trajectory.py` | 订阅 SPP 输出 → 流式 CSV | Step 1 加了 timestamp 列 |
| `scripts/make_map.py` | 早期 OSM 离线地图生成器 | Step 1 整合进 `generate_analysis.py` |
| `scripts/HOW_TO_RUN.md` | "Cat 接住，Bash 跑，Tee 存一份" 速查表 | 仍适用 |

---

## 经验教训（直接帮助了 Step 1）

1. **`pyubx2` 单位坑**：返回的 lat/lon 已经是十进制度数，不是原始整数 / 1e7（commit b2e60cc）
2. **`rosbags` 库 CDR 格式与 ROS1 不兼容**：必须改用原生 `import rosbag`（commit 005b0f6，Step 1 沿用）
3. **流式写入 CSV**：每个历元立刻 `flush` 到磁盘，Ctrl+C 不丢已记录数据
4. **NavSatFix `status` 字段必填**：缺失会导致 ROS1 反序列化失败（commit 9540a95）
5. **星历时间戳**：必须写在第一条 obs 之前，否则 bag 时长会错乱（commit f5b80eb）
6. **两路径互相验证**：云端 RTKLIB 与 ROS del1RTK 解出同一位置 → 链路无误，这一思路也是后续判断 UrbanNav 16km 误差是 bug 的依据
