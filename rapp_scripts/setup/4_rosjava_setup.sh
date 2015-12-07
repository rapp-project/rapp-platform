#!/bin/bash -ie

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

RosjavaPath="${HOME}/rapp_platform/rosjava"

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Rosjava \e[0m"

### [Download and install rosjava for KnowRob to work] ###
sudo apt-get update -qq &> /dev/null # Update aptitude package manager indexes.
sudo apt-get install -y python-wstool -qq &> /dev/null

# Create directory
if [ -d ${RosjavaPath} ]; then
  rm -rf ${RosjavaPath}
fi
mkdir -p ${RosjavaPath}

# Fetch repositories rosjava depends on, using the provided .rosinstall file.
echo -e "\e[1m\e[103m\e[31m [RAPP] Cloning Rosjava \e[0m"
wstool init -j4 ${RosjavaPath}/src rosjava.rosinstall &> /dev/null
cd ${RosjavaPath}

# Update rosdep with rosjava dependencies and install them.
rosdep update &> /dev/null
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y &> /dev/null

# Build rosjava
echo -e "\e[1m\e[103m\e[31m [RAPP] Building Rosjava \e[0m"
source /opt/ros/indigo/setup.bash --extend
catkin_make &> /dev/null

# Append into user's .bashrc
append="source ~/rapp_platform/rosjava/devel/setup.bash --extend"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# ROSJAVA\n${append}" \
  >> ~/.bashrc
append="export JAVA_HOME=/usr/lib/jvm/default-java"
grep -q "${append}" ~/.bashrc || echo -e "${append}" >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH' >> ~/.bashrc
