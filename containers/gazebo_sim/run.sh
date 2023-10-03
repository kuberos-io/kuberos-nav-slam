#!/bin/bash

docker run -it --rm \
    --env "NOVNC_ENABLE=true" \
    --env "WINDOW_MANAGER_ENABLE=true" \
    --env "STARTX11=true" \
    --name "gazebo_sim_tb4_cyclone" \
    -p 8082:8080 \
    --network ros-net \
    gazebo_sim_tb4_humble:latest \
    $*

    # --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \