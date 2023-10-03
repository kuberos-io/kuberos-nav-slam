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
# CycloneDDS
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Nav2 simple commmander
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-nav2-simple-commander && \
    rm -rf /var/lib/apt/lists/*

# python3 transforms3d
RUN apt-get update && \
    apt -y install python3-transforms3d && \
    rm -rf /var/lib/apt/lists/*

##############################################################
####                     Source Code                      ####
##############################################################
ADD src/nav_to_pose /ws/src/nav_to_pose

# Build
RUN cd /ws \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
    && rm -rf build
WORKDIR /ws


COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

CMD ros2 run nav_to_pose nav_to_pose use_sim_time:=True --launch-navigation True --localization-mode rtabmap --goal '0,-0.5,0;-2,-0.5,0;-3,-2.5,0;-0.3,-7,0'