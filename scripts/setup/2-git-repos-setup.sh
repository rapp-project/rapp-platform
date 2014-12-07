#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

# Step 2 : Github/ ROS repositories initialization

# Here you must add your public key to github for the clone command to be
# be able to execute, as the repos are private

#---------------------- RAPP Github repositories setup -----------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] Create Github folders \e[0m"
# Create folder for RAPP platform repo
cd ~/Desktop
mkdir rapp-platform-catkin-ws
cd rapp-platform-catkin-ws
mkdir src
cd src

echo -e "\e[1m\e[103m\e[31m [RAPP] HOP - Cloning the rapp-platform repo \e[0m"
# Clone the repository (public key should have been setup)
git clone git@github.com:rapp-project/rapp-platform.git

catkin_init_workspace

echo "source ~/Desktop/rapp-platform-catkin-ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-wstool
mkdir -p ~/Desktop/rosjava
wstool init -j4 ~/Desktop/rosjava/src https://raw.githubusercontent.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/rosjava.rosinstall
source /opt/ros/indigo/setup.bash
cd ~/Desktop/rosjava
rosdep update
rosdep install --from-paths src -i -y
catkin_make


echo -e "\e[1m\e[103m\e[31m [RAPP] Step 2 finished. Now the HOP setup must be performed. \e[0m"

