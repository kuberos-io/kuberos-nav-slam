#!/bin/bash -e
BASEDIR=$(dirname "$0")
ROS_DISTRO="humble"

echo "Using Dockerfile: $BASEDIR"
echo "From Context: $PWD"

# build image
DOCKER_BUILDKIT=1 docker build \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  $@ \
  -t rviz_${ROS_DISTRO}:latest \
  -f $BASEDIR/Dockerfile \
  $PWD
