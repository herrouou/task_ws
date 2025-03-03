#!/bin/bash
# defaukt humble
if [ -z "$ROS_DISTRO" ]; then
  echo "ROS_DISTRO 未设置，默认使用 humble"
  ROS_DISTRO=humble
else
  echo "check ROS_DISTRO: $ROS_DISTRO"
fi

# update
sudo apt-get update

# install
sudo apt-get install -y libcanberra-gtk-module libcanberra-gtk3-module \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-cv-bridge \
    python3-colcon-common-extensions
