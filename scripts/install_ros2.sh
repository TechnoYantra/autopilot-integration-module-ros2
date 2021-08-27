#!/bin/bash
echo "---------- $0 start ----------"
set -e
set -x

cd ~/

sudo apt-get install -y ros-foxy-nav2-* git \
    ros-foxy-tf-transformations \
    ros-foxy-mavros \
    ros-foxy-mavros-extras 

source /opt/ros/foxy/setup.bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/TechnoYantra/ros2-autopilot-integration-module.git
cd ..
colcon build --symlink-install
