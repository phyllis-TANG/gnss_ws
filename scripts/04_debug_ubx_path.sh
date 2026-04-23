#!/bin/bash
# 04_debug_ubx_path.sh — 诊断为什么 ublox_driver 打不开 UBX 文件
# 直接输出所有关键信息

echo "=========================================="
echo "  UBX 文件路径诊断"
echo "=========================================="
echo

WS=/root/gnss_ws
CONFIG=$WS/src/ublox_driver/config/driver_config.yaml
BAG_DIR=$WS/data/bags

echo "【A】当前 driver_config.yaml 完整内容"
echo "------------------------------------------"
cat "$CONFIG"
echo
echo

echo "【B】bags 目录里的所有文件（含软链接）"
echo "------------------------------------------"
ls -la "$BAG_DIR"
echo

echo "【C】提取配置里的 ubx_filepath"
echo "------------------------------------------"
CFG_PATH=$(grep '^ubx_filepath' "$CONFIG" | sed 's/.*ubx_filepath: *"\(.*\)".*/\1/')
echo "  config 里写的路径: [${CFG_PATH}]"
echo

echo "【D】测试这个路径"
echo "------------------------------------------"
if [ -e "$CFG_PATH" ]; then
    echo "  ✓ 文件/链接存在"
    ls -la "$CFG_PATH"
    if [ -L "$CFG_PATH" ]; then
        echo "  是软链接 → $(readlink -f "$CFG_PATH")"
        if [ -f "$(readlink -f "$CFG_PATH")" ]; then
            echo "  ✓ 链接目标存在，大小: $(stat -c%s "$(readlink -f "$CFG_PATH")") bytes"
        else
            echo "  ✗ 链接目标不存在！"
        fi
    fi
    # 试着用 C 风格的方式打开（跟 ublox_driver 的行为类似）
    python3 -c "
import os
p = '$CFG_PATH'
try:
    fd = os.open(p, os.O_RDONLY)
    size = os.lseek(fd, 0, 2)
    os.close(fd)
    print(f'  ✓ Python 以 O_RDONLY 打开成功，大小: {size} bytes')
except Exception as e:
    print(f'  ✗ Python open 失败: {e}')
"
else
    echo "  ✗ 路径不存在: [$CFG_PATH]"
fi
echo

echo "【E】所有真实 .ubx 文件（硬路径，无软链接）"
echo "------------------------------------------"
find "$BAG_DIR" -name "*.ubx" -type f 2>/dev/null | while read f; do
    echo "  $f  ($(stat -c%s "$f") bytes)"
done
echo

echo "【F】用 strace 捕获 ublox_driver 实际打开了哪些文件"
echo "------------------------------------------"
if command -v strace >/dev/null 2>&1; then
    echo "  启动 strace 模式，3 秒后自动结束..."
    timeout 3 strace -f -e trace=openat -o /tmp/ublox_strace.log \
        /root/gnss_ws/devel/lib/ublox_driver/ublox_driver _config_file:=$CONFIG 2>/dev/null || true
    echo "  抓取到的文件打开尝试（最后 20 条）："
    grep -E "\.ubx|\.yaml|driver_config" /tmp/ublox_strace.log | tail -20
else
    echo "  strace 未安装，跳过此项（不影响诊断）"
fi
echo

echo "【G】检查 config 文件是否有不可见字符（BOM、CRLF 等）"
echo "------------------------------------------"
file "$CONFIG"
echo
echo "  前 200 字节的十六进制："
head -c 200 "$CONFIG" | xxd | head -15
echo

echo "=========================================="
echo "  诊断完成"
echo "=========================================="
