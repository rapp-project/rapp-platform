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
#  Knowrob installation.
#  For more information regarding knowrob itself visit:
#    http://www.knowrob.org/
##


RappPlatformPath="${HOME}/rapp_platform"
RappPlatformFilesPath="${HOME}/rapp_platform_files"
KnowrobPath="${RappPlatformPath}/knowrob_catkin_ws"

# Check if the rapp_platform_files directory exists and create if not.
if [ ! -d "${RappPlatformFilesPath}" ]; then
  mkdir -p ${RappPlatformFilesPath}
fi

# Move the ontology file to the proper place
cp currentOntologyVersion.owl ${RappPlatformFilesPath}

# Download and install KnowRob
echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Knowrob \e[0m"
sudo apt-get install -y swi-prolog swi-prolog-java
sudo apt-get install -y ros-indigo-json-prolog-msgs
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-indigo-data-vis-msgs libjson-glib-dev
sudo apt-get install -y ros-indigo-rosjava-build-tools

append="export SWI_HOME_DIR=/usr/lib/swi-prolog"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Knowrob\n${append}"                        \
  >> ~/.bashrc


# Create directories to place knowrob catkin workspace
if [ -d "${KnowrobPath}" ]; then
  rm -rf ${KnowrobPath}
fi

mkdir -p ${KnowrobPath} && cd ${KnowrobPath}
mkdir src && cd src

# Initialize knowrob catkin workspace
catkin_init_workspace
cd ..
catkin_make
cd src

# Fetch knowrob sources
git clone https://github.com/rapp-project/knowrob.git
cd knowrob
git checkout indigo-devel
cd ../../

# Build knowrob
catkin_make

append="source ${KnowrobPath}/devel/setup.bash --extend"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Knowrob\n${append}"                        \
  >> ~/.bashrc
