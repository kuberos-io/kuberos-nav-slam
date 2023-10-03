#!/bin/bash

docker run -it --rm \
    --name "nav_to_pose_cyclone" \
    --network ros-net \
    nav_to_pose_humble:latest \
    $*

# --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \