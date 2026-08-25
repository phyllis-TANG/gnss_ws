# 运行 SPP 定位 — 直接复制粘贴

> 需要 2 个终端，都在 Docker 容器里

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

## 终端 2：下载 bag + 运行 SPP

**第一次运行先下载 bag（只需一次）：**

```bash
curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/spp_results/gnss_data.bag" \
     -o /root/gnss_ws/data/bags/gnss_data.bag
```

**运行 SPP：**

```bash
source /root/gnss_ws/devel/setup.bash
roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_ws/data/bags/gnss_data.bag \
  bag_rate:=1 \
  exclude_glonass:=true \
  rviz:=false
```

---

## ✅ 成功标志

终端 2 里出现每个历元的定位结果，例如：

```
[SPP] Epoch solved: lat=22.5349  lon=113.9372  alt=170.5m
```

---

## 实时查看定位坐标（可选，开第 3 个终端）

```bash
docker exec -it $(docker ps -q) bash
source /root/gnss_ws/devel/setup.bash
rostopic echo /gnss_spp_node/spp/navsatfix | grep -E "latitude|longitude|altitude"
```

---

## ❌ 常见报错处理

| 报错内容 | 原因 | 解决 |
|---------|------|------|
| `Only 0 sats pass elevation mask` | bag 文件是旧版（有 bug） | 重新执行上面的 curl 下载命令 |
| `roslaunch: command not found` | 没有 source | 先执行 `source /root/gnss_ws/devel/setup.bash` |
| `[rviz] process has died` | 容器没有显示器 | 加参数 `rviz:=false`（已包含在上面命令里） |
| `curl: not found` | 容器里没装 curl | 改用 `wget -O /root/gnss_ws/data/bags/gnss_data.bag "<url>"` |
