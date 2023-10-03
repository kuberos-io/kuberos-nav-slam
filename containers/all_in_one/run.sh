#!/bin/bash


docker run -it --rm \
    --env "NOVNC_ENABLE=true" \
    --env "WINDOW_MANAGER_ENABLE=true" \
    --env "STARTX11=true" \
    --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --name "nav2_kuberos_all_in_one" \
    --cpu-quota 0 \
    -p 8080:8080 \
    kuberos_nav_slam_all_in_one_humble:latest \
    $*

