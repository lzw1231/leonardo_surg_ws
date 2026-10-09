#!/bin/bash
#===============================================================================
# start-xpra.sh
# 功能：启动Xpra虚拟显示会话 :10，基于Xvfb，用于容器内GUI程序远程访问(RViz/Gazebo/Qt)
# 监听端口：14500(TCP)
# 分辨率：2560×1440@24bit（2K）
# 适用：DevContainer，ROS2开发环境
# 安全说明：无密码开放TCP接入，仅内网开发使用，禁止公网暴露
#===============================================================================
# 检查:10会话是否已存在，存在则直接退出
if xpra list 2>/dev/null | grep -q ":10"; then
    echo "[xpra] :10 session already running"
    exit 0
fi
# 初始化DBus session，GUI程序依赖
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    dbus-daemon --session --fork --print-address > /tmp/dbus-address 2>/dev/null
    [ -f /tmp/dbus-address ] && export DBUS_SESSION_BUS_ADDRESS=$(cat /tmp/dbus-address)
fi
echo "[xpra] starting session :10 ..."
# Xpra启动参数
xpra start :10 \
    --xvfb="Xvfb -screen 0 2560x1440x24 -nolisten tcp" \
    --daemon=yes \
    --no-pulseaudio \
    --notifications=no \
    --html=no \
    --tcp-auth=allow \
    --bind-tcp=0.0.0.0:14500 \
    --video-scaling=no \
    --encodings=rgb \
    >/tmp/xpra-start.log 2>&1
# 轮询检测会话启动状态，超时5s
for i in {1..5}; do
    sleep 1
    if xpra list 2>/dev/null | grep -q ":10"; then
        echo "[xpra] session :10 started (${i}s)"
        exit 0
    fi
done
# 脚本末尾失败分支
echo "[xpra] failed to start session :10"
cat /tmp/xpra-start.log 2>/dev/null
cat /run/xpra/10/server.log 2>/dev/null | tail -40
exit 1
