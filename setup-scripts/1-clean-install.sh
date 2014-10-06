#!/bin/bash 

# This script installs all the prerequisites and sets up all the needed 
# packages for RAPP platform to operate, initiating from a clean Ubuntu
# 14.04 install.

# Step 1 : Updates, ROS / HOP setup

#--------------------------------- Updates -----------------------------------#

# Refresh source lists
apt-get update
# Upgrades the current packages
apt-get upgrade

#-------------------------------- ROS setup ----------------------------------#

# Setup sources list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > \
  /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | \
  sudo apt-key add -

# Installation
apt-get update
apt-get -y install ros-indigo-desktop

# Initialize rosdep
rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

#------------------------ Auxiliary packages install -------------------------#

apt-get -y install openssh-server
apt-get -y install vim
apt-get -y install git gitg

#----------------------------- System trimming -------------------------------#

# Disable prompt from github to promote automation
echo -e "Host github.com\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config

#-------------------------------- HOP setup ----------------------------------#

# Create folders
mkdir ~/Desktop/hop && cd ~/Desktop/hop/

# Download bigloo and HOP
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2014-09-05/bigloo4.2a.tar
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2014-09-05/hop-3.0.0-pre11.tar

# Unzip them and remove the tars
tar -xvf bigloo4.2a.tar && rm bigloo4.2a.tar
tar -xvf hop-3.0.0-pre11.tar && rm hop-3.0.0-pre11.tar

# Install Bigloo
cd bigloo4.2a && ./configure && make && make install && ldconfig
cd ../hop-3.0.0-pre11 && ./configure && make && make install && ldconfig

# Initial setup of HOP must be performed through a web client.

 

