#!/bin/bash -e
if [ -z "${DISPLAY}" ]; then
  export DISPLAY=:0
fi

if [ -S "/tmp/.X11-unix/X${DISPLAY/:/}" ]; then
  echo "x11 already running..."
  return
fi

sudo mkdir -pm700 /tmp/runtime-user
sudo ln -snf /dev/ptmx /dev/tty7

Xvfb tty7 -noreset -dpi "${DPI}" +extension "RANDR" +extension "RENDER" +extension "MIT-SHM" -screen ${DISPLAY} ${SIZEW}x${SIZEH}x${CDEPTH} "${DISPLAY}" &

echo -n "Waiting for X socket..."
until [ -S "/tmp/.X11-unix/X${DISPLAY/:/}" ]; do sleep 1; done
echo "DONE"

if [ -n "${NOVNC_ENABLE}" ]; then
  echo "Starting VNC..."
  x11vnc -display "${DISPLAY}" -shared -forever -repeat -xkb -snapfb -threads -xrandr "resize" -rfbport 5900 -bg
  /opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 8080 --heartbeat 10 &
fi

if [ -n "${WINDOW_MANAGER_ENABLE}" ]; then
  echo "Starting Window Manager..."
  openbox &
fi

# force sw rendering
export LD_PRELOAD=/usr/local/lib/x86_64-linux-gnu/libGL.so.1
