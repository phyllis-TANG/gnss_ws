# 手把手教程：用 del1RTK 跑 GNSS SPP 定位

---

## 📋 开始前检查

你需要：
- ✅ 那台 Linux 机器（有 Docker 的那台）
- ✅ 至少打开 **3 个终端窗口**（或终端里的 3 个标签）

---

## 第一步：进入 Docker 容器

打开终端，输入：

```bash
docker ps
```

你会看到类似这样的输出：
```
CONTAINER ID   IMAGE   ...
e8d32d429cd8   ...
```

如果容器没有运行，先启动它（用你之前启动容器的命令）。

如果容器已经在跑，进入容器：
```bash
docker exec -it e8d32d429cd8 bash
```

> ✅ 成功标志：提示符变成 `root@e8d32d429cd8:~#`

---

## 第二步：修复 UBX 文件路径（只做一次）

在 Docker 容器里，复制粘贴这 3 条命令：

```bash
sed -i 's/^online:.*/online: 0/' \
  /root/gnss_ws/src/ublox_driver/config/driver_config.yaml

sed -i 's|ubx_filepath:.*|ubx_filepath: "/root/gnss_ws/data/bags/input.ubx"|' \
  /root/gnss_ws/src/ublox_driver/config/driver_config.yaml

grep "online\|ubx_filepath" /root/gnss_ws/src/ublox_driver/config/driver_config.yaml
```

> ✅ 成功标志：最后一条命令输出：
> ```
> online: 0
> ubx_filepath: "/root/gnss_ws/data/bags/input.ubx"
> ```

---

## 第三步：确认 UBX 文件存在

```bash
ls -lah /root/gnss_ws/data/bags/input.ubx
```

> ✅ 成功标志：显示文件大小（应该 > 1MB）
> ❌ 如果提示 No such file，运行这个找到真实文件：
> ```bash
> find /root/gnss_ws/data -name "*.ubx" | head -5
> ```
> 把找到的路径告诉 Claude。

---

## 第四步：设置环境变量（每次开新终端都要做）

```bash
source /root/gnss_ws/devel/setup.bash
```

> ✅ 没有任何输出 = 正常

---

## 第五步：开始运行（需要 3 个终端）

### 终端 1 ── 启动 ROS

```bash
source /root/gnss_ws/devel/setup.bash
roscore
```

> ✅ 成功标志：看到 `started core service [/rosout]`，然后光标停住不动（正常，它在等待）

---

### 终端 2 ── 启动 ublox_driver（读取 UBX 文件）

**新开一个终端，再次进入 Docker：**
```bash
docker exec -it e8d32d429cd8 bash
```

然后：
```bash
source /root/gnss_ws/devel/setup.bash
roslaunch ublox_driver ublox_driver.launch
```

> ✅ 成功标志：看到大量滚动的数字输出（卫星数据），类似：
> ```
> [ INFO] [xxx]: receive RAWX...
> ```
> ❌ 如果还是 "No such file"，把输出截图告诉 Claude

---

### 终端 3 ── 录制 ROS bag

**再开一个终端，再次进入 Docker：**
```bash
docker exec -it e8d32d429cd8 bash
```

然后：
```bash
source /root/gnss_ws/devel/setup.bash

rosbag record -O /root/gnss_ws/data/bags/gnss_data.bag \
  /ublox_driver/range_meas \
  /ublox_driver/ephem \
  /ublox_driver/glo_ephem \
  /ublox_driver/iono_params \
  /ublox_driver/receiver_lla \
  /ublox_driver/receiver_pvt
```

> ✅ 成功标志：看到 `[ INFO] Recording to .../gnss_data.bag`

**等终端 2 的数据输出停止后，在终端 3 按 Ctrl+C 停止录制。**

---

## 第六步：运行 SPP 定位

在终端 3 里，继续：

```bash
roslaunch del1RTK eval_spp.launch \
  bag:=/root/gnss_ws/data/bags/gnss_data.bag \
  bag_rate:=1 \
  exclude_glonass:=true
```

> ✅ 成功标志：
> - 终端里出现每个历元的定位输出
> - **在浏览器打开：`http://localhost:8888`** 能看到轨迹地图

---

## 🆘 遇到问题怎么办

| 问题 | 解决方法 |
|------|---------|
| 终端 2 报 `No such file` | 截图告诉 Claude |
| 终端 2 正常但终端 3 没有 `Recording` | 检查 roscore 是否还在跑 |
| `roslaunch del1RTK` 报错 | 截图告诉 Claude |
| 浏览器打不开 8888 | 正常，继续看终端输出即可 |

---

## 📌 每次开机需要重做的步骤

只需要做：第一步 → 第四步 → 第五步（三个终端）→ 第六步

第二步只需要做一次。

