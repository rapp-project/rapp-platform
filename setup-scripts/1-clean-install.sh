#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

# Step 1 : Updates, ROS / HOP setup

#--------------------------------- Updates -----------------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] Initiating source lists \e[0m"
# Refresh source lists
apt-get update

echo -e "\e[1m\e[103m\e[31m [RAPP] Upgrading packages \e[0m"
# Upgrades the current packages
apt-get upgrade

#-------------------------------- ROS setup ----------------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Setup sources list \e[0m"
# Setup sources list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > \
  /etc/apt/sources.list.d/ros-latest.list'

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Setup keys \e[0m"
# Setup keys
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | \
  sudo apt-key add -

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Installing \e[0m"
# Installation
apt-get update
apt-get -y install ros-indigo-desktop

echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Initializing rosdep \e[0m"
# Initialize rosdep
rosdep init
rosdep update

echo -e "\e[1m\e[103m\e[31m [RAPP] Setup ROS enironment \e[0m"
# Setup environment
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

#------------------------ Auxiliary packages install -------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing auxiliary packages \e[0m"
apt-get -y install openssh-server
apt-get -y install vim
apt-get -y install git gitg

#----------------------------- System trimming -------------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] Trimming system variables \e[0m"
# Disable prompt from github to promote automation
echo -e "Host github.com\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config

#-------------------------------- HOP setup ----------------------------------#

echo -e "\e[1m\e[103m\e[31m [RAPP] HOP - Create folders \e[0m"
# Create folders
mkdir ~/Desktop/hop && cd ~/Desktop/hop/

echo -e "\e[1m\e[103m\e[31m [RAPP] HOP - Download HOP and Bigloo \e[0m"
# Download bigloo and HOP
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2014-09-05/bigloo4.2a.tar
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2014-09-05/hop-3.0.0-pre11.tar

echo -e "\e[1m\e[103m\e[31m [RAPP] HOP - Unzipping... \e[0m"
# Unzip them and remove the tars
tar -xvf bigloo4.2a.tar && rm bigloo4.2a.tar
tar -xvf hop-3.0.0-pre11.tar && rm hop-3.0.0-pre11.tar

echo -e "\e[1m\e[103m\e[31m [RAPP] HOP - Installing bigloo \e[0m"
# Install Bigloo
cd bigloo4.2a && ./configure && make && make install && ldconfig

echo -e "\e[1m\e[103m\e[31m [RAPP] HOP - Installing HOP \e[0m"
# Install HOP
cd ../hop-3.0.0-pre11 && ./configure && make && make install && ldconfig

# Initial setup of HOP must be performed through a web client.

echo -e "\e[1m\e[103m\e[31m [RAPP] Step 1 complete. Now create the Github keys. \e[0m"


