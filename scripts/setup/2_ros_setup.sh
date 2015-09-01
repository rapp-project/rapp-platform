#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Setup sources list \e[0m"
# Setup sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > \
  /etc/apt/sources.list.d/ros-latest.list'

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Setup keys \e[0m"
# Setup keys
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | \
  sudo apt-key add -

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Installing \e[0m"
# Installation
sudo apt-get update
sudo apt-get install -y ros-indigo-desktop

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Initializing rosdep \e[0m"
# Initialize rosdep
sudo rosdep init
rosdep update

echo -e "\e[1m\e[103m\e[31m [RAPP] Setup ROS environment \e[0m"
# Setup environment
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y ros-indigo-rosbridge-server
