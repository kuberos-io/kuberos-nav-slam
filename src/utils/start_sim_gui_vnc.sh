#!/bin/bash

# wait for sim gui window

if [ ! -e /usr/bin/vinagre ]; then
  echo "ERROR: Could not find vnc client 'vinagre'. Please execute 'sudo apt install vinagre'"
  exit 1
fi

TIMEOUT=30
while [ $TIMEOUT != 0 ]; do
    WINID=$(DISPLAY=:10 xwininfo -name "Gazebo" 2>/dev/null | grep ^xwininfo | cut -d' ' -f 4)
    if [ -n "$WINID" ]; then
      break
    fi
      echo "Waiting for sim gui window id..."
    sleep 1
    TIMEOUT=`expr $TIMEOUT - 1`
done

if [ -z "$WINID" ]; then
  echo "No window id received after 30s"
  exit 1
fi
echo "Starting vnc for sim gui"
DISPLAY=:10 x11vnc -id $WINID -bg

DISPLAY=:50 vinagre $HOSTNAME:0 --vnc-scale >/dev/null 2>&1