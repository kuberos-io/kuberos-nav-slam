#!/bin/bash

docker run -it --rm \
    --name "nav2_rosbag_recording" \
    --network ros-net \
    -v $(pwd)/rosbags:/ws/rosbags \
    rosbag_recording_humble:latest \
    $*

# --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \