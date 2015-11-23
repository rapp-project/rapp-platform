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
#  Build Rapp Platform onto the system.
##

# Explicity check the result of each command and return with fail status if
# an error occures.
set -e

RappPlatformWs="${HOME}/rapp_platform/rapp-platform-catkin-ws"

# Install libzbar used by the qr_detection module.
sudo apt-get install -y libzbar-dev
source ~/.bashrc
sudo ldconfig

echo -e "\e[1m\e[103m\e[31m [RAPP] Create Github folders \e[0m"
# Create folder for RAPP platform repo
if [ -d "${RappPlatformWs}" ]; then
  rm -rf ${RappPlatformWs}
fi

mkdir -p ${RappPlatformWs} && cd ${RappPlatformWs}
mkdir src && cd src

# Initialize Rapp Platform catkin workspace
catkin_init_workspace

echo -e "\e[1m\e[103m\e[31m [RAPP] Cloning the rapp-platform repo \e[0m"
# Clone the repository (public key should have been setup)
git clone --recursive git@github.com:rapp-project/rapp-platform.git
git clone git@github.com:rapp-project/rapp-api.git

## [Fetch ]
cd rapp-api/python
# Insrall Python
sudo pip install -r dependencies.txt

# Append to user's .bashrc file.
append="source ~/rapp_platform/rapp-platform-catkin-ws/devel/setup.bash --extend"
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Rapp Platform\n${append}"                        \
  >> ~/.bashrc
source ~/.bashrc
echo "export PYTHONPATH=$PYTHONPATH:~/rapp_platform/rapp-platform-catkin-ws/src/rapp-api/python" >> ~/.bashrc
source ~/.bashrc

# catkin_make rapp-platform
cd ${RappPlatformWs} && catkin_make -j1

set +e
