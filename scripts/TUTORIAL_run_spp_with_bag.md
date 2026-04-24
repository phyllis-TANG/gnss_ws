# 手把手：下载 bag 文件直接跑 del1RTK SPP

> 前提：Docker 容器已启动，并且能访问 GitHub

---

## 第一步：进入 Docker 容器

```bash
docker ps
```
找到容器 ID，然后进入：
```bash
docker exec -it <容器ID> bash
```

---

## 第二步：下载 bag 文件（只做一次）

```bash
mkdir -p /root/gnss_ws/data/bags

curl -L "https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/spp_results/gnss_data.bag" \
     -o /root/gnss_ws/data/bags/gnss_data.bag
```

验证下载成功（应该显示约 801K）：
```bash
ls -lah /root/gnss_ws/data/bags/gnss_data.bag
```

---

## 第三步：运行 SPP（需要 2 个终端）

### 终端 1 ── 启动 roscore

```bash
docker exec -it <容器ID> bash
source /root/gnss_ws/devel/setup.bash
roscore
```

等到出现 `started core service [/rosout]` 后保持不动。

---

### 终端 2 ── 运行 SPP

```bash
docker exec -it <容器ID> bash
source /root/gnss_ws/devel/setup.bash

roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_ws/data/bags/gnss_data.bag \
  bag_rate:=1 \
  exclude_glonass:=true
```

---

## ✅ 成功标志

终端 2 里看到每个历元的定位输出，例如：

```
[gnss_spp_node] Epoch ... lat=22.53xx lon=113.93xx
```

---

## ❌ 常见问题

| 问题 | 解决方法 |
|------|---------|
| `curl` 下载失败 | 检查 Docker 容器能否访问 GitHub，或用手动下载 |
| `roslaunch` 找不到 `del1RTK` | 确认 `source /root/gnss_ws/devel/setup.bash` 已执行 |
| 没有定位输出 | 确认终端 1 的 roscore 还在运行 |
| 浏览器 8888 打不开 | 正常现象，看终端输出即可 |

---

## 手动下载 bag（如果 curl 不能用）

在你的 Windows 浏览器里打开：

```
https://github.com/phyllis-TANG/gnss_ws/raw/claude/review-gnss-spp-JmtgD/spp_results/gnss_data.bag
```

下载到本地后，用以下命令复制进 Docker：

```bash
docker cp gnss_data.bag <容器ID>:/root/gnss_ws/data/bags/gnss_data.bag
```
