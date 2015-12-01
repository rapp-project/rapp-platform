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
sudo apt-get install -qq -y swi-prolog swi-prolog-java
sudo apt-get install -qq -y ros-indigo-json-prolog-msgs
sudo apt-get install -qq -y python-rosinstall
sudo apt-get install -qq -y libjson-glib-dev
sudo apt-get install -qq -y ros-indigo-data-vis-msgs
sudo apt-get install -qq -y ros-indigo-rosjava-build-tools

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
catkin_init_workspace 1> /dev/null
cd ..
catkin_make 1> /dev/null
cd src

# Fetch knowrob sources
git clone https://github.com/rapp-project/knowrob.git 1> /dev/null
cd knowrob
git checkout indigo-devel 1> /dev/null
cd ../../

# Build knowrob
catkin_make 1> /dev/null

append="source ${KnowrobPath}/devel/setup.bash --extend"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Knowrob\n${append}"                        \
  >> ~/.bashrc
