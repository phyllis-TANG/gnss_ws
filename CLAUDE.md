# GNSS SPP Pipeline — 项目背景

## 研究目标
GNSS 信号反射建模（NLOS）+ LiDAR 密集点云。当前阶段：用 UrbanNav Medium-Urban-1 数据集跑通 del1RTK SPP，获得 SPP 轨迹与地面真值，作为 NLOS 分析的基准。

## 代码仓库
- GitHub: `phyllis-TANG/gnss_ws`，开发分支: `claude/review-gnss-spp-JmtgD`
- 本地工作区: `/home/user/gnss_ws`
- 关键脚本目录: `/home/user/gnss_ws/scripts/`

## Docker 环境
- 容器名: `ros1_gnss`（ROS1 Noetic）
- 容器内工作路径: `/root/`
- **每次进容器必须先执行**: `source /root/gnss_ws/devel/setup.bash`
- 宿主机用户: `ubuntu22`（注意：`/home/user` 在宿主机不存在，用 `/home/ubuntu22`）
- 进入容器: `sudo docker exec -it ros1_gnss bash`

## 数据集（UrbanNav Medium-Urban-1）
- obs 文件: `/root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs`
  - 时间: 2021年5月17日 02:33 起，约657个历元（~13分钟）
- nav 文件: `/root/urbannav_gnss/hksc137c.21n`（香港政府CORS，day 137 = 5月17日，匹配）
- 地面真值（GT）: UrbanNav 提供，格式为空格分隔文本，列: `UTCTime Week GPSTime Latitude Longitude H-Ell ...`，前两行为表头
- 大致位置: 香港，lat≈22.3198, lon≈114.2095, alt≈20m

## 脚本说明

### `rinex_to_rosbag.py`
RINEX obs + nav → ROS1 bag（供 del1RTK SPP 使用）

```bash
python3 /root/rinex_to_rosbag.py \
  --obs /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
  --nav /root/urbannav_gnss/hksc137c.21n \
  --out /root/gnss_urbannav.bag \
  --lat 22.3198 --lon 114.2095 --alt 20
```

输出 bag 包含：
- `/ublox_driver/range_meas` (GnssMeasMsg) — 伪距/载波相位/多普勒
- `/ublox_driver/ephem` (GnssEphemMsg) — GPS/GAL/BDS 星历
- `/ublox_driver/glo_ephem` (GnssGloEphemMsg) — GLONASS 星历
- `/ublox_driver/receiver_lla` (NavSatFix) — 静止参考点（仅用于 del1RTK 初始化）

**已解决的坑**：
- 必须用 `import rosbag`（原生ROS1），不能用 `pip install rosbags`（CDR格式不兼容）
- RINEX nav `brd[]` 索引：`[0]`=IODE, `[1]`=Crs, `[2]`=Δn, `[3]`=M0, `[4]`=Cuc, `[5]`=e, `[6]`=Cus, `[7]`=sqrtA, `[8]`=Toe, `[9]`=Cic, `[10]`=Ω0, `[11]`=Cis, `[12]`=i0, `[13]`=Crc, `[14]`=ω, `[15]`=Ω̇, `[16]`=i̇, `[18]`=GPS_week, `[21]`=health, `[22]`=TGD, `[23]`=IODC
- 星历时间戳写在第一个观测历元前1秒（否则 bag 时长会变成187天）
- **闰秒 Bug（已修复，commit b2022f8）**：RINEX 3 obs/nav 历元时间是 GPS 时间，但 `epoch_to_unix` 按 UTC 处理，再经 `unix_to_gps` 又加 18 秒，导致 bag 里 GPS TOW 比实际多 18s。卫星 toe 直接从 brd[8] 读取（正确），使得 del1RTK 计算 (t_obs - toe) 时多传播 18s（~70km 卫星位置偏差）→ SPP 偏 ~16km。修复：在 parse_obs 和 parse_nav 中各减 LEAP_SECONDS。

### `save_trajectory.py`
记录 SPP 定位结果到 CSV（含时间戳）

```bash
python3 /root/save_trajectory.py
```

输出: `/root/trajectory.csv`，列: `timestamp, lat, lon, alt_m, source`（source='spp'）

### `generate_analysis.py`
生成 SPP 分析 HTML 报告

```bash
# 无地面真值（仅 SPP 轨迹）
python3 /root/generate_analysis.py

# 有地面真值（SPP vs RTK/INS GT，时间最近邻匹配，容限5秒）
python3 /root/generate_analysis.py --gt /root/urbannav_gt.txt
```

输出: `/root/spp_analysis.html`，拷到宿主机：
```bash
sudo docker cp ros1_gnss:/root/spp_analysis.html ~/spp_analysis_urbannav.html
```

## SPP 完整运行步骤（3个终端）

```bash
# 终端1：roscore
source /root/gnss_ws/devel/setup.bash && roscore

# 终端2：记录轨迹
source /root/gnss_ws/devel/setup.bash && python3 /root/save_trajectory.py

# 终端3：运行 SPP
source /root/gnss_ws/devel/setup.bash
roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_urbannav.bag \
  bag_rate:=1 \
  exclude_glonass:=false \
  rviz:=false
# bag 播完后回到终端2按 Ctrl+C
```

## del1RTK 订阅的 Topic
- `/ublox_driver/range_meas` → GnssMeasMsg
- `/ublox_driver/ephem` → GnssEphemMsg
- `/ublox_driver/glo_ephem` → GnssGloEphemMsg
- `/ublox_driver/receiver_lla` → NavSatFix（接收机大致位置）
- 输出: `/gnss_spp_node/spp/navsatfix`

## 相关包路径
- del1RTK: `/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del1RTK/`
- gnss_comm msgs: `/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/gnss_comm/msg/`
- rinex_utils / compute_spp (备用纯Python路径): `/root/gnss_ws/PSRI-73-2309-PR-Dev/rospak/src/del2AINLOS/scripts/`

## 卫星编号约定（gnss_comm）
- GPS: PRN (1–32)
- GLONASS: 32 + PRN
- Galileo: 59 + PRN
- BDS: 97 + PRN

## 时间转换
- GPS_EPOCH_UNIX = 315964800（1980-01-06 UTC）
- 2021年闰秒 = 18秒
- GPS time = Unix time − 315964800 + 18

## SPP 基准结果（UrbanNav Medium-Urban-1，2021-05-17，香港）
- 轨迹点数: 635（SPP），GT 787 点，时间匹配 621/635
- **SPP–GT 均值: 84.8 m，RMS: 103.8 m，95th: 176.1 m**
- 符合香港城市峡谷 GNSS-only SPP 预期（50–200m）

## 待完成
- [x] 跑通 SPP + GT 对比，误差 <200m ✓
- [ ] 分析 SPP 误差分布（Urban Canyon 多径/NLOS 成分）
- [ ] 下载 del2AINLOS 所需基站 obs（同时段香港 CORS），运行 NLOS 分类
- [ ] 结合 LiDAR 点云做 NLOS 建模
