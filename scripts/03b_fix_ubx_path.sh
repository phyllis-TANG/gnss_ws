#!/bin/bash
# 脚本 03b：修复 UBX 文件路径问题（括号/空格导致打不开）
# 策略：创建软链接 input.ubx 指向原文件，然后更新 config

set -e

WS=/root/gnss_ws
CONFIG=$WS/src/ublox_driver/config/driver_config.yaml
BAG_DIR=$WS/data/bags

echo "=========================================="
echo "  修复 UBX 文件路径 (特殊字符问题)"
echo "=========================================="
echo

echo "【1】列出当前的 UBX 文件"
echo "------------------------------------------"
ls -la "$BAG_DIR"/*.ubx 2>/dev/null
echo

echo "【2】检查当前 config 里的 ubx_filepath"
echo "------------------------------------------"
grep "ubx_filepath" "$CONFIG"
echo

echo "【3】测试当前 config 里的路径能不能打开"
echo "------------------------------------------"
CURRENT_PATH=$(grep "ubx_filepath" "$CONFIG" | sed 's/.*ubx_filepath: *"\(.*\)".*/\1/')
echo "  路径: $CURRENT_PATH"
if [ -f "$CURRENT_PATH" ]; then
    echo "  ✓ 文件存在 (bash 能找到)"
    echo "  大小: $(stat -c%s "$CURRENT_PATH") bytes"
else
    echo "  ✗ 文件不存在 —— 这就是 ublox_driver 报错的原因！"
fi
echo

echo "【4】选一个干净的 UBX 文件源"
echo "------------------------------------------"
# 按优先级找：先找无特殊字符的，再找带特殊字符的
SRC=""
# 优先用 input.ubx（如果已经存在且不是 0 字节）
if [ -s "$BAG_DIR/input.ubx" ]; then
    SRC="$BAG_DIR/input.ubx"
    echo "  使用已有的 input.ubx"
else
    # 找第一个真实的 ubx 文件
    for f in "$BAG_DIR"/*.ubx; do
        if [ -f "$f" ] && [ -s "$f" ]; then
            SRC="$f"
            break
        fi
    done
fi

if [ -z "$SRC" ]; then
    echo "  ✗ 找不到任何有效的 UBX 文件"
    exit 1
fi
echo "  源文件: $SRC ($(stat -c%s "$SRC") bytes)"
echo

echo "【5】创建干净路径的软链接"
echo "------------------------------------------"
CLEAN_PATH="$BAG_DIR/clean_input.ubx"
# 如果不是它自己，建链接
if [ "$SRC" != "$CLEAN_PATH" ]; then
    ln -sf "$SRC" "$CLEAN_PATH"
    echo "  ✓ 创建软链接: $CLEAN_PATH → $SRC"
else
    echo "  源文件本身就是干净路径"
fi
ls -la "$CLEAN_PATH"
echo

echo "【6】更新 driver_config.yaml"
echo "------------------------------------------"
cp "$CONFIG" "${CONFIG}.bak2"
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
ubx_filepath: "${CLEAN_PATH}"
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
echo "  ✓ 配置已更新"
echo "  ubx_filepath: $CLEAN_PATH"
echo

echo "【7】最终验证"
echo "------------------------------------------"
grep "ubx_filepath\|online" "$CONFIG"
echo
if [ -f "$CLEAN_PATH" ] || [ -L "$CLEAN_PATH" ]; then
    echo "  ✓ 软链接/文件存在"
    echo "  ls: $(ls -la "$CLEAN_PATH")"
else
    echo "  ✗ 软链接创建失败"
fi
echo

echo "=========================================="
echo "  修复完成，重新在终端2跑："
echo "  source /root/gnss_ws/devel/setup.bash"
echo "  roslaunch ublox_driver ublox_driver.launch"
echo "=========================================="
