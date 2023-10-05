ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

ARG APT_MIRROR
ARG ROS_APT_MIRROR

SHELL ["/bin/bash", "-xo", "pipefail", "-c"]

RUN if [ -n "${APT_MIRROR}" ]; then \
    sed -i -r "s;http://security.ubuntu.com/ubuntu/;[trusted=yes] ${APT_MIRROR};g" /etc/apt/sources.list && \
    sed -i -r "s;http://archive.ubuntu.com/ubuntu/;[trusted=yes] ${APT_MIRROR};g" /etc/apt/sources.list && \
    sed -i -n '/multiverse/!p' /etc/apt/sources.list\
    ;fi

RUN apt update \
    && apt -y install --no-install-recommends wget unzip curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN if [ -n "${ROS_APT_MIRROR}" ]; then \
    sed -i -r "s;http://packages.ros.org/ros2/ubuntu;[arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_MIRROR};g"  /etc/apt/sources.list.d/ros2-latest.list \
    ;fi

RUN apt-get update && apt-get upgrade --no-install-recommends -q -y \
    && rm -rf /var/lib/apt/lists/*


##############################################################
####                     ROS Packages                     ####
##############################################################
# CycloneDDS; nav2_bringup, rtabmap_ros
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
                   ros-${ROS_DISTRO}-rtabmap-ros \
                   ros-${ROS_DISTRO}-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*


##############################################################
####                     Source Code                      ####
##############################################################
ADD src/nav2_bringup_extended /ws/src/nav2_bringup_extended

# Build
RUN cd /ws \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
    && rm -rf build
WORKDIR /ws


COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

CMD ros2 launch nav2_bringup_extended nav2_bringup_extended.launch.py use_sim_time:=True use_composition:=False autostart:=False localization:=rtabmap params_file:=/ws/src/nav2_bringup_extended/params/params.yaml
