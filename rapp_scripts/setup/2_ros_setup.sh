#!/bin/bash

##
# MIT License (MIT)

# Copyright (c) <2014> <Rapp Project EU>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Authors: Manos Tsardoulias
# Contact: etsardou@iti.gr
##

##
#  Installation of ros-indigo-desktop.
##


# Setup sources list
echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Setup sources list \e[0m"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > \
  /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Setup keys \e[0m"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | \
  sudo apt-key add -

# Installation
echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Installing \e[0m"
sudo apt-get update
sudo apt-get install -y ros-indigo-desktop

# Initialize rosdep
echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Initializing rosdep \e[0m"
sudo rosdep init
rosdep update

# Setup environment
echo -e "\e[1m\e[103m\e[31m [RAPP] Setup ROS environment \e[0m"
append="source /opt/ros/indigo/setup.bash"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Load ROS environment variables\n${append}" \
  >> ~/.bashrc
source ~/.bashrc

# Install rosbridge_server. This will allow third party clients (web clients)
# to connect to ROS.
sudo apt-get install -y ros-indigo-rosbridge-server
sudo apt-get install -y ros-indigo-global-planner
sudo apt-get install -y ros-indigo-map-server
