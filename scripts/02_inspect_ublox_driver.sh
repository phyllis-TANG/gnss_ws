#!/bin/bash
# 诊断脚本 02：检查 ublox_driver 包的使用方式
# 用法：bash 02_inspect_ublox_driver.sh
# 只读，不改动

set +e

echo "=========================================="
echo "  ublox_driver 包使用方式诊断"
echo "=========================================="
echo

UBX_DRIVER=/root/gnss_ws/src/ublox_driver

echo "【1】ublox_driver 包目录结构"
echo "------------------------------------------"
if [ -d "$UBX_DRIVER" ]; then
    ls -la "$UBX_DRIVER"
    echo
    echo "子目录："
    find "$UBX_DRIVER" -maxdepth 2 -type d | sort
else
    echo "  找不到 $UBX_DRIVER"
fi
echo

echo "【2】launch 文件"
echo "------------------------------------------"
find "$UBX_DRIVER" -name "*.launch" 2>/dev/null | while read f; do
    echo "----- $f -----"
    cat "$f"
    echo
done
echo

echo "【3】config/yaml 文件"
echo "------------------------------------------"
find "$UBX_DRIVER" \( -name "*.yaml" -o -name "*.yml" \) 2>/dev/null | while read f; do
    echo "----- $f -----"
    cat "$f"
    echo
done
echo

echo "【4】README / 说明文档"
echo "------------------------------------------"
find "$UBX_DRIVER" -maxdepth 2 \( -iname "readme*" -o -iname "*.md" \) 2>/dev/null | while read f; do
    echo "----- $f -----"
    head -80 "$f"
    echo "..."
    echo
done
echo

echo "【5】可执行节点（check CMakeLists）"
echo "------------------------------------------"
if [ -f "$UBX_DRIVER/CMakeLists.txt" ]; then
    grep -E "add_executable|install.*TARGETS" "$UBX_DRIVER/CMakeLists.txt" | head -20
fi
echo
echo "编译产出的节点："
find /root/gnss_ws/devel/lib/ublox_driver -maxdepth 1 -type f -executable 2>/dev/null
echo

echo "【6】关键消息类型"
echo "------------------------------------------"
grep -rE "GnssMeasMsg|GnssEphemMsg|GnssGloEphemMsg|GnssPVTSolnMsg|StampedFloat64Array" \
    "$UBX_DRIVER/src" 2>/dev/null | head -10
echo

echo "【7】UBX 文件位置"
echo "------------------------------------------"
find /root -maxdepth 5 \( -name "*.ubx" -o -name "*.UBX" \) 2>/dev/null
echo

echo "【8】检查 ublox_driver 支持的输入模式（串口 vs 文件）"
echo "------------------------------------------"
grep -rnE "input_type|input_mode|from_file|file_path|log_file|ubx_file|input_file" \
    "$UBX_DRIVER/src" "$UBX_DRIVER/launch" "$UBX_DRIVER/config" 2>/dev/null | head -20
echo

echo "=========================================="
echo "  诊断完成"
echo "=========================================="
