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
sudo apt-get upgrade

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

sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" > \
  /etc/apt/sources.list'
  
sudo apt-get update
sudo apt-get install -y hop

echo -e "\e[1m\e[103m\e[31m [RAPP] Step 1 complete. Now create the Github keys. \e[0m"


