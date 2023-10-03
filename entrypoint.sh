#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

if [ -n "$ROS_SERVICES" ]; then
  if [ "$RMW_IMPLEMENTATION" == "rmw_cyclonedds_cpp" ]; then
      /usr/bin/ros2_cyclonedds_config_generator.py -s $ROS_SERVICES
      export CYCLONEDDS_URI=/etc/cyclonedds.xml
  fi
fi

# x11
if [ -n "$STARTX11" ]; then
  source /startx11_virtual.sh
fi

# gazebo
if [ -n "$IGN_SERVICES" ]; then
  IGN_EXPORTS=$(/usr/bin/get_ign_exports.py)
  echo $IGN_EXPORTS
  eval $IGN_EXPORTS
fi

exec "$@"
