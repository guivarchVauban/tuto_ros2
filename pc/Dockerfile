FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    iproute2 \
    iputils-ping \
    nano \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/root/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
