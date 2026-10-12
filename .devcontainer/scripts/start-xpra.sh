#!/bin/bash
#===============================================================================
# start-xpra.sh
# 功能：启动Xpra虚拟显示会话 :10，基于Xvfb，用于容器内GUI程序远程访问(RViz/Gazebo/Qt)
# 监听端口：14500(TCP)
# 分辨率：2560×1440@24bit（2K）
# 适用：DevContainer，ROS2开发环境
# 安全说明：无密码开放TCP接入，仅内网开发使用，禁止公网暴露
#===============================================================================

set -u

echo "[xpra] ===== start-xpra.sh begin ====="

#-------------------------------------------------------------------------------
# 1. 清理上一次的残留（进程、锁文件、状态目录、dbus pid）
#    这是关键：容器重启后 /tmp/.X10-lock、/run/xpra 等残留会让 Xvfb 起不来
#-------------------------------------------------------------------------------
echo "[xpra] cleaning stale state ..."
pkill -9 -f 'xpra start' 2>/dev/null
pkill -9 -f 'Xvfb-for-Xpra' 2>/dev/null
pkill -9 -f 'dbus-daemon --session' 2>/dev/null
sleep 1

rm -rf /run/xpra \
       /root/.xpra \
       /tmp/.X10-lock \
       /tmp/.X11-unix/X10 \
       /tmp/10 \
       /tmp/xpra-start.log \
       /tmp/dbus-address \
       /run/dbus/pid 2>/dev/null

#-------------------------------------------------------------------------------
# 2. 初始化 DBus session（GUI 程序依赖）
#-------------------------------------------------------------------------------
if [ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]; then
    dbus-daemon --session --fork --print-address > /tmp/dbus-address 2>/dev/null
    if [ -s /tmp/dbus-address ]; then
        export DBUS_SESSION_BUS_ADDRESS="$(cat /tmp/dbus-address)"
        echo "[xpra] DBus session: $DBUS_SESSION_BUS_ADDRESS"
    else
        echo "[xpra] Warning: dbus-daemon failed to start (GUI 通知功能可能不可用)"
    fi
fi

#-------------------------------------------------------------------------------
# 3. 启动 Xpra 会话
#    关键参数说明：
#      --tcp-auth=none   无密码接入（原 allow 需要密码文件，会导致认证失败）
#      --html=no         容器里没装 xpra-html5，关掉避免报错
#      --daemon=yes      后台运行
#-------------------------------------------------------------------------------
echo "[xpra] starting session :10 ..."

xpra start :10 \
    --xvfb="Xvfb -screen 0 2560x1440x24 -nolisten tcp" \
    --daemon=yes \
    --no-pulseaudio \
    --notifications=no \
    --html=no \
    --tcp-auth=none \
    --bind-tcp=0.0.0.0:14500 \
    --video-scaling=no \
    --encodings=rgb \
    >/tmp/xpra-start.log 2>&1

#-------------------------------------------------------------------------------
# 4. 等待会话真正 LIVE（原逻辑只看 list 里有没有 :10，
#    但 UNKNOWN / defunct 也会匹配到，导致误判成功）
#-------------------------------------------------------------------------------
for i in {1..10}; do
    sleep 1
    if xpra list 2>/dev/null | grep -qE "LIVE session at :10"; then
        echo "[xpra] session :10 started (${i}s)"
        echo "[xpra] ===== start-xpra.sh OK ====="
        exit 0
    fi
done

#-------------------------------------------------------------------------------
# 5. 失败分支：打印所有可能日志，便于定位
#-------------------------------------------------------------------------------
echo "[xpra] failed to start session :10"
echo "----- /tmp/xpra-start.log -----"
cat /tmp/xpra-start.log 2>/dev/null
echo "----- /run/xpra/10/server.log -----"
cat /run/xpra/10/server.log 2>/dev/null | tail -40
echo "----- /tmp/10/server.log -----"
cat /tmp/10/server.log 2>/dev/null | tail -40
exit 1