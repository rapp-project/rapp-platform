#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

# Step 1 : Updates, ROS / HOP setup

#--------------------------------- Updates -----------------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] Initiating source lists \e[0m"
# Refresh source lists
sudo apt-get update

echo -e "\e[1m\e[103m\e[31m [RAPP] Upgrading packages \e[0m"
# Upgrades the current packages
sudo apt-get upgrade -y

#-------------------------------- ROS setup ----------------------------------#

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

echo -e "\e[1m\e[103m\e[31m [RAPP] Setup ROS enironment \e[0m"
# Setup environment
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

#------------------------ Auxiliary packages install -------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing auxiliary packages \e[0m"
sudo apt-get install -y openssh-server
sudo apt-get install -y vim
sudo apt-get install -y git gitg

#----------------------------- System trimming -------------------------------#

#echo -e "\e[1m\e[103m\e[31m [RAPP] Trimming system variables \e[0m"
# Disable prompt from github to promote automation
#echo -e "Host github.com\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config

#-------------------------------- HOP setup ----------------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP by deb \e[0m"

sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" >> \
  /etc/apt/sources.list'
  
sudo apt-get update
sudo apt-get install -y hop

echo -e "\e[1m\e[103m\e[31m [RAPP] Step 1 complete. Now create the Github keys. \e[0m"

#-------------------------------Github repos setup----------------------------#

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

echo "source ~/Desktop/rapp-platform-catkin-ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc

# Download and install rosjava for KnowRob to work
sudo apt-get install python-wstool
mkdir -p ~/Desktop/rosjava
wstool init -j4 ~/Desktop/rosjava/src https://raw.githubusercontent.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/rosjava.rosinstall
source /opt/ros/indigo/setup.bash
cd ~/Desktop/rosjava
rosdep update
rosdep install --from-paths src -i -y
catkin_make
echo "source ~/Desktop/rosjava/devel/setup.bash --extend" >> ~/.bashrc
echo "export JAVA_HOME=/usr/lib/jvm/default-java" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc

# Download and install KnowRob
sudo apt-get install -y swi-prolog swi-prolog-java
echo "export SWI_HOME_DIR=/usr/lib/swi-prolog" >> ~/.bashrc
source ~/.bashrc
cd ~/Desktop
mkdir knowrob_catkin_ws
cd knowrob_catkin_ws
mkdir src
cd src
catkin_init_workspace
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-indigo-data-vis-msgs libjson-glib-dev
git clone https://github.com/knowrob/knowrob.git
cd knowrob
git checkout indigo-devel
cd ../../
catkin_make
echo "source ~/Desktop/knowrob_catkin_ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc

# catkin_make rapp-platform
cd ~/Desktop/rapp-platform-catkin-ws
catkin_make

echo -e "\e[1m\e[103m\e[31m [RAPP] Step 2 finished. Now the HOP setup must be performed. \e[0m"

