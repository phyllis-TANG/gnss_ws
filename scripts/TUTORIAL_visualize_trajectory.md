# 查看 SPP 轨迹地图 — 直接复制粘贴

> 需要 3 个终端，都在 Docker 容器里

---

## 准备：进入 Docker 容器（每个终端都要做）

```bash
docker exec -it $(docker ps -q) bash
```

---

## 终端 1：启动 roscore

```bash
source /root/gnss_ws/devel/setup.bash
roscore
```

看到 `started core service [/rosout]` 后不要关闭。

---

## 终端 2：下载记录脚本 + 开始记录轨迹

```bash
curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/scripts/save_trajectory.py" \
     -o /root/gnss_ws/save_trajectory.py

source /root/gnss_ws/devel/setup.bash
python3 /root/gnss_ws/save_trajectory.py
```

看到 `开始记录，跑完 bag 后按 Ctrl+C 生成地图文件...` 后保持不动。

---

## 终端 3：运行 SPP

```bash
source /root/gnss_ws/devel/setup.bash
roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_ws/data/bags/gnss_data.bag \
  bag_rate:=1 \
  exclude_glonass:=true \
  rviz:=false
```

等终端 3 的 bag 播放完毕（看到 `[rosbag_player] Done.`）。

---

## 生成地图文件

回到**终端 2**，按 **Ctrl+C**。

会显示：
```
CSV 已保存: /root/gnss_ws/spp_results/spp_trajectory.csv
HTML 已保存: /root/gnss_ws/spp_results/trajectory.html
把这个文件复制到 Windows，用浏览器打开即可看到地图！
```

---

## 把 HTML 拷到 Windows（在容器外的终端运行）

```bash
docker cp $(docker ps -q):/root/gnss_ws/spp_results/trajectory.html ~/Desktop/trajectory.html
```

然后在 Windows 用浏览器打开 `trajectory.html`，即可看到带 OpenStreetMap 背景的轨迹地图。

---

## ❌ 常见问题

| 问题 | 解决方法 |
|------|---------|
| 终端 2 显示 `没有收到 SPP 数据` | 确认终端 3 有正常的定位输出再按 Ctrl+C |
| `docker cp` 找不到文件 | 先运行 `ls /root/gnss_ws/spp_results/` 确认文件存在 |
| 地图打开后是空白 | 需要联网加载 OpenStreetMap 瓦片，检查 Windows 网络 |
| 地图打开后看不到轨迹 | 用浏览器开发者工具（F12）查看 Console 有无报错 |
