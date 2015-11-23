#!/bin/bash -ie

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

RosjavaPath="${HOME}/rapp_platform/rosjava"

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Rosjava \e[0m"

### [Download and install rosjava for KnowRob to work] ###
sudo apt-get update # Update aptitude package manager indexes.
sudo apt-get install -y python-wstool

# Create directory
if [ -d ${RosjavaPath} ]; then
  rm -rf ${RosjavaPath}
fi
mkdir -p ${RosjavaPath}

# Fetch repositories rosjava depends on, using the provided .rosinstall file.
wstool init -j4 ${RosjavaPath}/src rosjava.rosinstall
cd ${RosjavaPath}

# Update rosdep with rosjava dependencies and install them.
rosdep update
rosdep install --from-paths src -i -y

# Build rosjava
catkin_make

# Append into user's .bashrc
append="source ~/rapp_platform/rosjava/devel/setup.bash --extend"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# ROSJAVA\n${append}" \
  >> ~/.bashrc
append="export JAVA_HOME=/usr/lib/jvm/default-java"
grep -q "${append}" ~/.bashrc || echo -e "${append}" >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH' >> ~/.bashrc
