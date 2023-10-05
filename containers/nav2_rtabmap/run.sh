#!/bin/bash

docker run -it --rm \
    --name "nav2_rtabmap_cyclone" \
    --network ros-net \
    nav2_rtabmap_humble:latest \
    $*

# --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \