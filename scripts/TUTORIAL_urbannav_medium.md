# UrbanNav Medium-Urban-1 — 从零到 SPP 分析（复制粘贴版）

> 数据集：UrbanNav-HK-Medium-Urban-1（香港中度城市峡谷，33.7 GB）  
> 工具：del1RTK SPP + gnss_comm + ROS1 Noetic

---

## 准备：文件放在哪里

在宿主机上建好目录：

```bash
mkdir -p ~/datasets/urbannav/medium-urban-1/gnss
mkdir -p ~/datasets/urbannav/medium-urban-1/ground_truth
mkdir -p ~/datasets/urbannav/medium-urban-1/bag
```

把下载好的文件放进去：
- RINEX obs/nav 文件 → `gnss/`
- Ground Truth 文件 → `ground_truth/`
- ROS bag → `bag/`

---

## 第一步：把 RINEX 文件复制进 Docker

```bash
# 在宿主机终端运行（把文件名换成你实际下载的）
sudo docker cp ~/datasets/urbannav/medium-urban-1/gnss/ ros1_gnss:/root/urbannav_gnss/
```

---

## 第二步：进入 Docker，安装依赖，下载转换脚本

```bash
sudo docker exec -it ros1_gnss bash
```

进入容器后：

```bash
pip install georinex pandas

curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/scripts/rinex_to_rosbag.py" \
     -o /root/rinex_to_rosbag.py
```

---

## 第三步：查看 RINEX 文件名，运行转换

```bash
ls /root/urbannav_gnss/
```

把 obs 文件和 nav 文件路径填入下面命令：

```bash
python3 /root/rinex_to_rosbag.py \
  --obs /root/urbannav_gnss/<obs文件名> \
  --nav /root/urbannav_gnss/<nav文件名> \
  --out /root/gnss_urbannav.bag \
  --lat 22.3 --lon 114.2 --alt 50
```

看到 `完成！` 和测量历元数后继续。

---

## 第四步：跑 SPP（需要 3 个终端）

**终端 1 — roscore：**
```bash
sudo docker exec -it ros1_gnss bash
source /root/gnss_ws/devel/setup.bash
roscore
```

**终端 2 — 记录轨迹：**
```bash
sudo docker exec -it ros1_gnss bash
source /root/gnss_ws/devel/setup.bash

curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/scripts/save_trajectory.py" \
     -o /root/save_trajectory.py

python3 /root/save_trajectory.py
```

**终端 3 — 运行 SPP：**
```bash
sudo docker exec -it ros1_gnss bash
source /root/gnss_ws/devel/setup.bash
roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_urbannav.bag \
  bag_rate:=1 \
  exclude_glonass:=false \
  rviz:=false
```

等终端 3 播完（显示 `Done.`），回到终端 2 按 `Ctrl+C`。

---

## 第五步：生成分析报告

```bash
# 在容器里
curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/scripts/generate_analysis.py" \
     -o /root/generate_analysis.py

python3 /root/generate_analysis.py
```

把 HTML 拷到宿主机：

```bash
# 在宿主机终端
sudo docker cp ros1_gnss:/root/spp_analysis.html ~/spp_analysis_urbannav.html
```

浏览器打开：`file:///home/ubuntu22/spp_analysis_urbannav.html`

---

## 常见问题

| 问题 | 解决 |
|------|------|
| `georinex` 安装失败 | `pip install georinex --index-url https://pypi.org/simple/` |
| obs 文件里没有 C1C | 改用 `--obs` 文件名里带 `_MO` 的那个 |
| 测量历元数为 0 | 检查 obs 文件是否是 RINEX 3.02 格式（文件头第一行） |
| del1RTK 没有输出 | 先试 `exclude_glonass:=true`，排除 GLONASS 干扰 |
| bag 找不到 | 检查路径，用 `ls -lh /root/gnss_urbannav.bag` 确认 |
