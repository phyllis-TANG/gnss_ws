#!/bin/bash
# 05_clean_setup.sh — 一键重置：复制 UBX 到干净路径 + 重写 config
# 放弃所有软链接/旧 config，从零开始

set -e

WS=/root/gnss_ws
CONFIG=$WS/src/ublox_driver/config/driver_config.yaml
CLEAN_DIR=/root/ubx_data
CLEAN_UBX=$CLEAN_DIR/input.ubx

echo "=========================================="
echo "  一键重置 UBX 配置"
echo "=========================================="
echo

# ── ① 找一个真实的 UBX 源文件 ─────────────────
echo "【1】搜索可用的 UBX 文件..."
SRC=""
# 按优先级搜索多个可能位置
for dir in "$WS/data/bags" "$WS/data" "$WS" "/root" "/tmp"; do
    [ ! -d "$dir" ] && continue
    while IFS= read -r f; do
        # 必须是真实文件（不是软链接且 > 1KB）
        if [ -f "$f" ] && [ ! -L "$f" ] && [ "$(stat -c%s "$f")" -gt 1024 ]; then
            SRC="$f"
            break 2
        fi
    done < <(find "$dir" -maxdepth 3 -name "*.ubx" -o -name "*.UBX" 2>/dev/null)
done

if [ -z "$SRC" ]; then
    echo "  ✗ 整个系统都找不到真实的 UBX 文件"
    echo "  请把 UBX 文件放到 $WS/data/bags/ 下再重跑"
    exit 1
fi
echo "  ✓ 找到源文件：$SRC"
echo "  大小：$(stat -c%s "$SRC") bytes ($(( $(stat -c%s "$SRC") / 1024 / 1024 )) MB)"
echo

# ── ② 创建干净目录并复制 ─────────────────────
echo "【2】创建干净目录并复制文件..."
mkdir -p "$CLEAN_DIR"
# 删掉旧的（如果有）
[ -e "$CLEAN_UBX" ] && rm -f "$CLEAN_UBX"
cp "$SRC" "$CLEAN_UBX"
echo "  ✓ 复制完成：$CLEAN_UBX"
ls -la "$CLEAN_UBX"
echo

# ── ③ 用 Python 验证能打开 ────────────────────
echo "【3】验证文件可以被打开（模拟 ublox_driver 的行为）..."
python3 -c "
import os
p = '$CLEAN_UBX'
fd = os.open(p, os.O_RDONLY)
size = os.lseek(fd, 0, 2)
os.close(fd)
print(f'  ✓ open() 成功，大小 = {size} bytes')
"
echo

# ── ④ 备份旧 config 并重写 ────────────────────
echo "【4】重写 driver_config.yaml..."
if [ -f "$CONFIG" ]; then
    cp "$CONFIG" "${CONFIG}.bak_$(date +%s)"
fi

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
ubx_filepath: "${CLEAN_UBX}"
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
echo "  ✓ 已重写 $CONFIG"
echo

# ── ⑤ 显示新 config ──────────────────────────
echo "【5】新 config 内容："
echo "------------------------------------------"
cat "$CONFIG"
echo "------------------------------------------"
echo

# ── ⑥ 最终提示 ────────────────────────────────
echo "=========================================="
echo "  ✅ 重置完成！现在去跑终端2："
echo "=========================================="
echo
echo "  source /root/gnss_ws/devel/setup.bash"
echo "  roslaunch ublox_driver ublox_driver.launch"
echo
echo "  应该看到 ublox_driver 正常运行，不再报 No such file"
echo
echo "=========================================="
