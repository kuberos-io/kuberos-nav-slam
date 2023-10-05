ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

ARG APT_MIRROR
ARG ROS_APT_MIRROR
ARG NOVNC_VERSION=1.3.0

SHELL ["/bin/bash", "-xo", "pipefail", "-c"]


ENV DEBIAN_FRONTEND=noninteractive 

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


##############################################################
####                        VNC                           ####
##############################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
        x11vnc && \
    rm -rf /var/lib/apt/lists/* && \
    curl -fsSL "https://github.com/novnc/noVNC/archive/v${NOVNC_VERSION}.tar.gz" | tar -xzf - -C /opt && \
    mv -f "/opt/noVNC-${NOVNC_VERSION}" /opt/noVNC && \
    ln -snf /opt/noVNC/vnc.html /opt/noVNC/index.html && \
    # Use the latest Websockify source to expose noVNC
    git clone "https://github.com/novnc/websockify.git" /opt/noVNC/utils/websockify


##############################################################
####             Setup Software Rendering                 ####
##############################################################
RUN apt-get update && \
    apt install -y --no-install-recommends \
    xvfb \
    openbox && \
    rm -rf /var/lib/apt/lists/* 

RUN apt-get update && \
    apt -y install llvm bison flex scons libdrm-dev libxcb-glx0-dev libx11-xcb-dev libxcb-dri2-0-dev libelf-dev libxcb-xfixes0-dev meson wget python3-mako libxext-dev libxcb-randr0-dev libxrandr-dev && \
    rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    wget https://archive.mesa3d.org/mesa-21.3.6.tar.xz && \
    tar xf mesa-21.3.6.tar.xz && \
    rm mesa-21.3.6.tar.xz && \
    cd mesa-21.3.6 && \
    mkdir build && \
    cd build && \
    meson -D glx=gallium-xlib -D gallium-drivers=swrast -D platforms=x11 -D dri3=false -D dri-drivers="" -D vulkan-drivers="" -D buildtype=release -D optimization=3 -D prefix=/usr/local && \
    ninja && \
    ninja install && \
    rm -rf /tmp/mesa-21.3.6

RUN apt -y remove bison flex scons meson python3-mako && \
    apt -y autoremove

RUN echo "#!/bin/sh" > /usr/bin/x2goglx2
RUN echo "LD_PRELOAD=/usr/local/lib/x86_64-linux-gnu/libGL.so.1 exec \"\$@\"" >> /usr/bin/x2goglx2
RUN chmod +x /usr/bin/x2goglx2
RUN echo "/usr/local/lib/x86_64-linux-gnu" > /etc/ld.so.conf.d/libgl.conf
RUN ldconfig

COPY src/utils/startx11_virtual.sh /
COPY src/utils/check_ign_running.sh /
COPY src/utils/get_ign_exports.py /usr/bin


##############################################################
####                     ROS Packages                     ####
##############################################################
# CycloneDDS
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Gazebo, Turtlebot4 simulator, python3-transform3d
RUN apt-get update && \
    apt install -y --no-install-recommends \
    ignition-fortress \
    ros-${ROS_DISTRO}-turtlebot4-simulator \
    python3-transforms3d && \
    rm -rf /var/lib/apt/lists/*

# RTABmap
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-rtabmap-ros && \
    rm -rf /var/lib/apt/lists/*


##############################################################
####                     Source Code                      ####
##############################################################
ADD src /ws/src
ADD .ignition /root/.ignition

# RUN mkdir -p /ws/src
# RUN --mount=type=bind,target=/ws/src \
#   cd /ws \
#   && source /opt/ros/${ROS_DISTRO}/setup.bash \
#   && colcon build \
#   && rm -rf build

# Build
RUN cd /ws \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
    && rm -rf build
WORKDIR /ws

# ADD private_datasets/dataset_realrobot /ws/datasets/dataset_realrobot
# ADD private_datasets/rgbd_dataset_freiburg2_pioneer_slam /ws/datasets/rgbd_dataset_freiburg2_pioneer_slam

COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

# Environment variables
ENV DISPLAY :0
ENV XDG_RUNTIME_DIR /tmp/runtime-user
ENV SIZEW 1600
ENV SIZEH 1200
ENV DPI 96
ENV CDEPTH 24
ENV QT_X11_NO_MITSHM 1

# CMD x2goglx2 ros2 launch nav2_edge_eval nav2_edge_eval.launch.py record_rosbag:=true world:=warehouse
CMD x2goglx2 ros2 launch nav2_edge_all_in_one_launch nav2_edge_eval.launch.py record_rosbag:=true world:=warehouse
