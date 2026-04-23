#!/bin/bash
# 诊断脚本 01：检查 ROS 环境和项目结构
# 用法：bash 01_diagnose_env.sh
# 不做任何修改，只是读取信息

set +e  # 不因单条命令失败而退出

echo "=========================================="
echo "  GNSS SPP 环境诊断脚本 (只读，不改动)"
echo "=========================================="
echo

echo "【1】系统环境"
echo "------------------------------------------"
echo "主机名        : $(hostname)"
echo "是否在 Docker : $(grep -q docker /proc/1/cgroup 2>/dev/null && echo "是" || ([ -f /.dockerenv ] && echo "是" || echo "否"))"
echo "Ubuntu 版本   : $(lsb_release -d 2>/dev/null | cut -f2)"
echo "当前用户      : $(whoami)"
echo "当前目录      : $(pwd)"
echo

echo "【2】ROS 环境"
echo "------------------------------------------"
echo "ROS 版本      : $(rosversion -d 2>/dev/null || echo '未检测到 ROS')"
echo "ROS_DISTRO    : ${ROS_DISTRO:-未设置}"
echo "ROS_ROOT      : ${ROS_ROOT:-未设置}"
echo "roscore 路径  : $(which roscore 2>/dev/null || echo '找不到')"
echo

echo "【3】工作空间结构"
echo "------------------------------------------"
WS=/root/gnss_ws
echo "工作空间路径  : $WS"
if [ -d "$WS" ]; then
    echo "子目录列表    :"
    ls -la "$WS" | awk 'NR>1 {print "  " $NF}' | grep -v '^  \.$\|^  \.\.$'
else
    echo "  工作空间不存在！"
fi
echo

echo "【4】src 下的内容"
echo "------------------------------------------"
if [ -d "$WS/src" ]; then
    ls -la "$WS/src" | awk 'NR>1 {print "  " $NF}' | grep -v '^  \.$\|^  \.\.$'
else
    echo "  $WS/src 不存在！"
fi
echo

echo "【5】查找所有 package.xml（确定 ROS 包位置）"
echo "------------------------------------------"
if [ -d "$WS/src" ]; then
    find "$WS/src" -name "package.xml" 2>/dev/null | while read f; do
        pkg=$(grep -oP '(?<=<name>)[^<]+' "$f" | head -1)
        echo "  $pkg  ←  $(dirname $f)"
    done
fi
echo

echo "【6】catkin 识别到的包（关键）"
echo "------------------------------------------"
if [ -f "$WS/devel/setup.bash" ]; then
    source "$WS/devel/setup.bash"
    echo "  已 source devel/setup.bash"
    for pkg in del1RTK gnss_comm novatel_msgs ublox_driver rviz_osm del2AINLOS del3CRTK del4MSF del5CAS; do
        path=$(rospack find "$pkg" 2>/dev/null)
        if [ -n "$path" ]; then
            echo "  ✓ $pkg: $path"
        else
            echo "  ✗ $pkg: 未找到"
        fi
    done
else
    echo "  没找到 devel/setup.bash，工作空间尚未编译"
fi
echo

echo "【7】关键依赖库"
echo "------------------------------------------"
for lib in Eigen3 GTSAM GeographicLib; do
    result=$(apt list --installed 2>/dev/null | grep -i "libeigen3\|libgtsam\|libgeographiclib" | head -3)
done
echo "  Eigen3      : $(dpkg -l libeigen3-dev 2>/dev/null | awk '/^ii/{print $3}' || echo '未安装')"
echo "  GTSAM       : $(ls /usr/local/lib/libgtsam.so* 2>/dev/null | head -1 || dpkg -l libgtsam-dev 2>/dev/null | awk '/^ii/{print $3}' || echo '未找到')"
echo "  GeographicLib: $(dpkg -l libgeographic-dev 2>/dev/null | awk '/^ii/{print $3}' || dpkg -l libgeographiclib-dev 2>/dev/null | awk '/^ii/{print $3}' || echo '未找到')"
echo

echo "【8】git 状态"
echo "------------------------------------------"
if [ -d "$WS/.git" ]; then
    cd "$WS"
    echo "  当前分支  : $(git branch --show-current 2>/dev/null)"
    echo "  远程仓库  :"
    git remote -v | sed 's/^/    /'
else
    echo "  $WS 不是 git 仓库"
fi
echo

echo "=========================================="
echo "  诊断完成！请把以上输出发给 Claude"
echo "=========================================="
