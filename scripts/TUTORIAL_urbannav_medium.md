# UrbanNav Medium-Urban-1 — SPP 分析完整步骤

> 数据集：UrbanNav-HK-Medium-Urban-1（香港中度城市峡谷）  
> 工具：del1RTK SPP + gnss_comm + ROS1 Noetic  
> 所有命令可直接复制粘贴

---

## 前提：你已经下载好了

- `UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs`（来自 GNSS 下载项）
- 星历文件（来自香港政府 CORS 或 IGS，后缀 `.nav` 或 `.rnx`）
- Ground Truth 文件（来自 Ground Truth 下载项）
- ROS bag（33.7 GB，后台下载中或已完成）

---

## 第一步：宿主机建立目录，放好文件

```bash
mkdir -p ~/datasets/urbannav/medium-urban-1/gnss
mkdir -p ~/datasets/urbannav/medium-urban-1/ground_truth
mkdir -p ~/datasets/urbannav/medium-urban-1/bag
```

把文件手动移动到对应目录：
- obs 文件 + nav/rnx 文件 → `gnss/`
- ground truth 文件 → `ground_truth/`
- bag 文件（下完后）→ `bag/`

---

## 第二步：把 GNSS 文件复制进 Docker

```bash
sudo docker cp ~/datasets/urbannav/medium-urban-1/gnss ros1_gnss:/root/urbannav_gnss
```

确认复制成功：

```bash
sudo docker exec ros1_gnss ls /root/urbannav_gnss/
```

---

## 第三步：进入 Docker，下载转换脚本

```bash
sudo docker exec -it ros1_gnss bash
```

进入容器后：

```bash
curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/scripts/rinex_to_rosbag.py" \
     -o /root/rinex_to_rosbag.py
```

---

## 第四步：运行转换（把文件名换成你的实际文件名）

```bash
python3 /root/rinex_to_rosbag.py \
  --obs /root/urbannav_gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.obs \
  --nav /root/urbannav_gnss/<你的nav文件名> \
  --out /root/gnss_urbannav.bag \
  --lat 22.3 --lon 114.2 --alt 50
```

> `--lat/--lon/--alt` 是香港的大致坐标，不需要精确，仅用于卫星仰角计算。

看到以下输出表示成功：

```
[1/3] 解析 obs: ...   找到 XXX 个历元
[2/3] 解析 nav: ...   找到 XXX 条星历
[3/3] 写入 bag: ...
完成！输出: /root/gnss_urbannav.bag
```

---

## 第五步：跑 SPP（需要 3 个终端）

### 终端 1 — 启动 roscore

```bash
sudo docker exec -it ros1_gnss bash
source /root/gnss_ws/devel/setup.bash
roscore
```

看到 `started core service [/rosout]` 后不动。

---

### 终端 2 — 开始记录轨迹

```bash
sudo docker exec -it ros1_gnss bash
source /root/gnss_ws/devel/setup.bash

curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/scripts/save_trajectory.py" \
     -o /root/save_trajectory.py

python3 /root/save_trajectory.py
```

看到 `记录中` 后不动。

---

### 终端 3 — 运行 SPP

```bash
sudo docker exec -it ros1_gnss bash
source /root/gnss_ws/devel/setup.bash

roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_urbannav.bag \
  bag_rate:=1 \
  exclude_glonass:=false \
  rviz:=false
```

等待 bag 播完（显示 `Done.`），回到**终端 2** 按 `Ctrl+C`。

---

## 第六步：生成分析报告

在容器里（任意终端）：

```bash
curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/scripts/generate_analysis.py" \
     -o /root/generate_analysis.py

python3 /root/generate_analysis.py
```

把 HTML 拷到宿主机：

```bash
# 在宿主机终端
sudo docker cp ros1_gnss:/root/spp_analysis.html ~/spp_analysis_urbannav.html
```

浏览器打开：

```
file:///home/ubuntu22/spp_analysis_urbannav.html
```

---

## 常见问题

| 现象 | 解决方法 |
|------|---------|
| 历元数为 0 | obs 文件头部格式不对，发给助手确认 |
| 星历数为 0 | nav 文件路径或格式有误，确认后缀是 `.nav`/`.rnx` |
| del1RTK 没有定位输出 | 先加 `exclude_glonass:=true` 再试 |
| 轨迹点数很少（< 50） | 正常，Urban Canyon 卫星少；或 obs 时段较短 |
| HTML 地图底图不显示 | 正常（OSM 国内受限），轨迹线仍可见 |
