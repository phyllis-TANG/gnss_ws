# Step 1 — SPP 基准线建立（UrbanNav Medium-Urban-1）

**目标**：用 del1RTK 跑通 GNSS-only SPP，获得 SPP 轨迹与地面真值对比，作为后续 NLOS 分析的基准。

---

## 数据准备清单

- [x] **obs 文件**：`UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs`（RINEX 3.03，GPS 时间系，混合星座）
- [x] **nav 文件**：`hksc137c.21n`（香港政府 CORS，RINEX 3.02，GPS-only，day 137 = 2021-05-17）
- [x] **地面真值**：`UrbanNav_TST_GT_raw.txt`（RTK/INS，DMS 坐标格式，787 个点）
- [x] **Docker 容器**：`ros1_gnss`（ROS1 Noetic + del1RTK + gnss_comm）

---

## 运行步骤

### 1. RINEX → ROS bag

```bash
python3 /root/rinex_to_rosbag.py \
  --obs /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
  --nav /root/urbannav_gnss/hksc137c.21n \
  --out /root/gnss_urbannav.bag \
  --lat 22.3198 --lon 114.2095 --alt 20
```

验证：`rostopic echo -n 1 --bag /root/gnss_urbannav.bag /ublox_driver/range_meas | grep tow`  
期望：`tow: 95593.xxx`（= 2021-05-17 02:33 GPS TOW，不含多余 18s）

### 2. 运行 SPP（3 个终端）

```bash
# 终端 1
source /root/gnss_ws/devel/setup.bash && roscore

# 终端 2
source /root/gnss_ws/devel/setup.bash && python3 /root/save_trajectory.py

# 终端 3
source /root/gnss_ws/devel/setup.bash
roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_urbannav.bag bag_rate:=1 \
  exclude_glonass:=false rviz:=false
# bag 播完后回终端 2 按 Ctrl+C
```

### 3. 生成分析报告

```bash
python3 /root/generate_analysis.py --gt /root/urbannav_gt.txt
# 宿主机取出
sudo docker cp ros1_gnss:/root/spp_analysis.html ~/spp_analysis_urbannav.html
```

---

## 本阶段修复的 Bug

### Bug 1 — 闰秒导致 GPS TOW 偏 +18s（主因，~16km 位置误差）

**现象**：SPP 结果偏离真值 16km，海拔 −5000m  
**根因**：RINEX 3 obs/nav 历元用 GPS 时间记录，但 `epoch_to_unix()` 按 UTC 处理，再经 `unix_to_gps()` 加 18s，导致 bag 里 `range_meas.tow` 比真实值多 18s。卫星 toe 从星历直接读取（正确），(t\_obs − toe) 多 18s → 卫星位置偏 ~70km → WLS 收敛到错误位置。  
**修复**：`rinex_to_rosbag.py` 的 `parse_obs` 和 `parse_nav` 中各减 `LEAP_SECONDS`：

```python
t = epoch_to_unix(year, month, day, hour, minute, sec) - LEAP_SECONDS
```

**commit**：`b2022f8`

---

### Bug 2 — HTML 报告 JS 语法错误（地图和图表不显示）

**现象**：浏览器报 `Uncaught SyntaxError: missing ) after argument list`，左侧地图和误差曲线空白  
**根因**：`generate_analysis.py` 中 Chart.js 配置的 f-string 末尾多了一对 `}}` → 渲染后多出一个 `}`，破坏 JS 语法  
**修复**：删除多余的 `}}` 对  
**commit**：`3581d8f`

---

## 最终结果

| 指标 | 数值 |
|------|------|
| SPP 轨迹点数 | 635 |
| GT 点数 | 787 |
| 时间匹配点数 | 621 / 635（容限 5s） |
| **SPP–GT 均值** | **84.8 m** |
| **SPP–GT RMS** | **103.8 m** |
| **SPP–GT 95th** | **176.1 m** |

结论：符合香港城市峡谷 GNSS-only SPP 预期（50–200 m），基准线建立成功。

---

## 关键技术说明

| 项目 | 说明 |
|------|------|
| nav 文件格式 | RINEX 3.02（非 RINEX 2，`.n` 后缀不代表 RINEX 2）|
| obs 时间系统 | GPS 时间（头部 `TIME OF FIRST OBS ... GPS`）|
| GT 坐标格式 | DMS（度分秒）：`22 18 04.31949 114 10 44.60559` = 22.3012°N, 114.179°E |
| 时间匹配方式 | SPP UTC 时间戳与 GT UTCTime 列最近邻匹配 |
| del1RTK SPP 算法 | WLS（加权最小二乘），非 GTSAM；初始点来自 `receiver_lla` |

---

## 下一步

- [ ] 分析 SPP 误差时序：识别高误差历元（NLOS/多径）
- [ ] 下载同时段香港 CORS 基站 obs，运行 del2AINLOS NLOS 分类
- [ ] 结合 LiDAR 点云建立 3D 遮挡模型，验证 NLOS 标注
