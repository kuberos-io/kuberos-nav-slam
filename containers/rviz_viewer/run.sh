#!/bin/bash

docker run -it --rm \
    --env "NOVNC_ENABLE=true" \
    --env "WINDOW_MANAGER_ENABLE=true" \
    --env "STARTX11=true" \
    --name "rviz_viewer" \
    -p 8082:8080 \
    --network ros-net \
    rviz_humble:latest \
    $*
