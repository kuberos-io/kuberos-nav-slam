#!/bin/bash

docker run -it --rm \
    --name "nav2_stack" \
    --network ros-net \
    nav2_stack_humble:latest \
    $*