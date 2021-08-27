#!/bin/bash
echo "---------- $0 start ----------"
set -e
set -x

cd ~/

sudo apt-get install -y git \
    ros-noetic-mavros \
    ros-noetic-mavros-extras 

source /opt/ros/noetic/setup.bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh 
sudo ./install_geographiclib_datasets.sh
pip3 install pymavlink

mkdir -p ~/catkin_ws/src
git clone https://github.com/TechnoYantra/mavros_node.git
cd ~/catkin_ws/src
cd ..
catkin_make 