#!/bin/bash -i

##

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

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
sudo apt-get update -qq &> /dev/null
sudo apt-get install -qq -y ros-indigo-desktop &> /dev/null

# Initialize rosdep
echo -e "\e[1m\e[103m\e[31m [RAPP] ROS - Initializing rosdep \e[0m"
sudo rosdep init &> /dev/null
rosdep update &> /dev/null

# Setup environment
echo -e "\e[1m\e[103m\e[31m [RAPP] Setup ROS environment \e[0m"
append="source /opt/ros/indigo/setup.bash --extend"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Load ROS environment variables\n${append}" \
  >> ~/.bashrc

# Install rosbridge_server. This will allow third party clients (web clients)
# to connect to ROS.
sudo apt-get install -qq -y ros-indigo-rosbridge-server &> /dev/null
sudo apt-get install -qq -y ros-indigo-global-planner &> /dev/null
sudo apt-get install -qq -y ros-indigo-map-server &> /dev/null
