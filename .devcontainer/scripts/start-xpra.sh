#!/bin/bash
if xpra list 2>/dev/null | grep -q ":10"; then
    echo "[xpra] :10 already running"
    exit 0
fi

if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    dbus-daemon --session --fork --print-address > /tmp/dbus-address 2>/dev/null
    [ -f /tmp/dbus-address ] && export DBUS_SESSION_BUS_ADDRESS=$(cat /tmp/dbus-address)
fi

echo "[xpra] starting :10 ..."
xpra start :10 \
    --xvfb="Xvfb -screen 0 1920x1080x24 -nolisten tcp" \
    --daemon=yes \
    --no-pulseaudio \
    --notifications=no \
    --html=no \
    --bind-tcp=0.0.0.0:14500 \
    --tcp-auth=password:changeme \
    >/tmp/xpra-start.log 2>&1

for i in 1 2 3 4 5; do
    sleep 1
    if xpra list 2>/dev/null | grep -q ":10"; then
        echo "[xpra] :10 started (after ${i}s)"
        exit 0
    fi
done

echo "[xpra] failed to start :10"
cat /tmp/xpra-start.log 2>/dev/null
cat /run/xpra/10/server.log 2>/dev/null | tail -40
exit 1
