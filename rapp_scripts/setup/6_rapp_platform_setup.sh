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
#  Build Rapp Platform onto the system.
##


RappPlatformWs="${HOME}/rapp_platform/rapp-platform-catkin-ws"
RappWebServicesPkgPath="${RappPlatformWs}/src/rapp-platform/rapp_web_services"

# Install libzbar used by the qr_detection module.
sudo apt-get install -qq -y libzbar-dev &> /dev/null
sudo ldconfig &> /dev/null

echo -e "\e[1m\e[103m\e[31m [RAPP] Create Github folders \e[0m"
# Create folder for RAPP platform repo
if [ -d "${RappPlatformWs}" ]; then
  rm -rf ${RappPlatformWs}
fi

mkdir -p ${RappPlatformWs} && cd ${RappPlatformWs}
mkdir src && cd src

# Initialize Rapp Platform catkin workspace
echo -e "\e[1m\e[103m\e[31m [RAPP] Initializing Rapp Catkin Workspace\e[0m"
catkin_init_workspace &> /dev/null

# Clone the repository (public key should have been setup)
echo -e "\e[1m\e[103m\e[31m [RAPP] Cloning the rapp-platform repo, branch: $1\e[0m"
git clone --recursive --branch=$1 https://github.com/rapp-project/rapp-platform.git &> /dev/null
git clone https://github.com/rapp-project/rapp-api.git &> /dev/null

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing pip dependencies\e[0m"
cd rapp-api/python
# Insrall Python
sudo pip install -r dependencies.txt &> /dev/null

# Append to user's .bashrc file.
append="source ~/rapp_platform/rapp-platform-catkin-ws/devel/setup.bash --extend"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Rapp Platform\n${append}"                        \
  >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:~/rapp_platform/rapp-platform-catkin-ws/src/rapp-api/python' >> ~/.bashrc

# catkin_make rapp-platform
echo -e "\e[1m\e[103m\e[31m [RAPP] Initializing Rapp Platform\e[0m"
cd ${RappPlatformWs} && catkin_make

# Install rapp_web_services package deps
cd ${RappWebServicesPkgPath}
if [ -n "${TRAVIS_BRANCH}" ]; then
  echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Travis npm dependencies\e[0m"
  sudo npm install
else
  echo -e "\e[1m\e[103m\e[31m [RAPP] Installing npm dependencies\e[0m"
  npm install
fi
