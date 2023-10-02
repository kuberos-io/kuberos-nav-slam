ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
# Install libraries for using nvidia GPU
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    mesa-utils \
    libgl1-mesa-glx \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev &&\
    rm -rf /var/lib/apt/lists/*


##############################################################################
##              Install Ignition Fortress and TurtleBot                     ##
##############################################################################

# Install ignition
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update && \
    apt install -y -qq --no-install-recommends \
    wget && \
    rm -rf /var/lib/apt/lists/
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&\
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - &&\
    DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt install -y -qq --no-install-recommends \
    ignition-fortress &&\
    rm -rf /var/lib/apt/lists/*

# Turtlebot4
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update \
    && apt install -y -qq --no-install-recommends \
    ros-humble-rqt* \
    python3-transforms3d \
    ros-humble-turtlebot4-simulator &&\
    rm -rf /var/lib/apt/lists/*

# CycloneDDS
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# RTABMap
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-rtabmap-ros && \
    rm -rf /var/lib/apt/lists/*

# Python pip
RUN apt-get update && \
    apt -y install python3-pip && \
    rm -rf /var/lib/apt/lists/*

# RTabMap Dataset Evaluation 
RUN apt-get update && \
    apt -y install ros-${ROS_DISTRO}-image-proc \
    ros-${ROS_DISTRO}-imu-complementary-filter && \
    rm -rf /var/lib/apt/lists/*

##############################################################################
##                           Create Non-root User                           ##
##############################################################################
ARG USER=ros
ARG PASSWORD=ros
ARG UID=1000
ARG GID=1000
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src


# Python and libraries (Install with non-root user)
ADD requirements.txt /tmp
# RUN pip3 --no-cache-dir install -r /tmp/requirements.txt


##############################################################################
##                                 Build ROS and run                        ##
##############################################################################

WORKDIR /home/$USER/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc
RUN echo "source /workspace/install/setup.bash" >> /home/$USER/.bashrc
RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ADD .ignition /home/${USER}/.ignition
RUN sudo chown -R ros:ros /home/${USER}/.ignition/