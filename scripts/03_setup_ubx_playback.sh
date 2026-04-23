#!/bin/bash
# 脚本 03：配置 ublox_driver 读取 UBX 文件
# 运行后会告诉你接下来在 3 个终端里敲什么

set -e

WS=/root/gnss_ws
CONFIG=$WS/src/ublox_driver/config/driver_config.yaml
BAG_DIR=$WS/data/bags

echo "=========================================="
echo "  配置 ublox_driver 文件回放模式"
echo "=========================================="
echo

# ── 找 UBX 文件 ──────────────────────────────
echo "【1】查找 UBX 文件..."
UBX_FILE=""
for f in "$BAG_DIR"/*.ubx "$BAG_DIR"/*.UBX; do
    [ -f "$f" ] && UBX_FILE="$f" && break
done

if [ -z "$UBX_FILE" ]; then
    echo "  ✗ 在 $BAG_DIR 里找不到 .ubx 文件"
    echo "  请把 UBX 文件放到 $BAG_DIR/ 目录下"
    exit 1
fi
echo "  ✓ 找到：$UBX_FILE"
echo

# ── 备份并修改 config ──────────────────────────
echo "【2】更新 ublox_driver 配置..."
cp "$CONFIG" "${CONFIG}.bak"
echo "  备份原始 config → ${CONFIG}.bak"

cat > "$CONFIG" << YAML
%YAML:1.0

# input options
online: 0
input_serial_port: "/dev/ttyACM0"
serial_baud_rate: 115200
input_rtcm: 0
rtcm_tcp_port: 3503
config_receiver_at_start: 0
receiver_config_filepath: "~/catkin_ws/src/ublox_driver/config/ublox_rcv_config.yaml"
ubx_filepath: "${UBX_FILE}"
rtk_correction_ecef:
    rows: 3
    cols: 1
    dt: d
    data: [ 0, 0, 0 ]

# output options
to_ros: 1
to_file: 0
dump_dir: "~/tmp/ublox_driver_test/"
to_serial: 0
output_serial_port: "/dev/ttyACM0"
YAML

echo "  ✓ 配置已更新：online=0，ubx_filepath 指向 UBX 文件"
echo

# ── 输出 bag 路径 ─────────────────────────────
BAG_OUT="$BAG_DIR/gnss_spp_input.bag"

# ── 打印操作步骤 ──────────────────────────────
echo "=========================================="
echo "  现在打开 3 个终端，按顺序执行："
echo "=========================================="
echo
echo "【终端 1】启动 ROS master（先跑这个）"
echo "------------------------------------------"
echo "  source /root/gnss_ws/devel/setup.bash"
echo "  roscore"
echo
echo "【终端 2】启动 ublox_driver（读取 UBX 文件）"
echo "------------------------------------------"
echo "  source /root/gnss_ws/devel/setup.bash"
echo "  roslaunch ublox_driver ublox_driver.launch"
echo
echo "【终端 3】录制 ROS bag（等终端2有数据输出后再跑）"
echo "------------------------------------------"
echo "  source /root/gnss_ws/devel/setup.bash"
echo "  rosbag record -O $BAG_OUT \\"
echo "    /ublox_driver/range_meas \\"
echo "    /ublox_driver/ephem \\"
echo "    /ublox_driver/glo_ephem \\"
echo "    /ublox_driver/iono_params \\"
echo "    /ublox_driver/receiver_lla \\"
echo "    /ublox_driver/receiver_pvt"
echo
echo "  ← ublox_driver 结束后，终端3 按 Ctrl+C 停止录制"
echo
echo "=========================================="
echo "  录制完成后，运行 SPP："
echo "=========================================="
echo
echo "  source /root/gnss_ws/devel/setup.bash"
echo "  roslaunch del1RTK eval_spp.launch \\"
echo "    bag:=$BAG_OUT \\"
echo "    bag_rate:=1 \\"
echo "    exclude_glonass:=true"
echo
echo "  然后打开浏览器访问 http://localhost:8888 看轨迹地图"
echo
echo "=========================================="
echo "  配置完成！按上面步骤操作即可"
echo "=========================================="
