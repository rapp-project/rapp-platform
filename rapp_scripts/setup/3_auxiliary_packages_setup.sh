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
#  Install required external auxiliary packages.
##


echo -e "\e[1m\e[103m\e[31m [RAPP] Installing auxiliary packages \e[0m"
# Allow remote secure connections to the RAPP-Platform.
sudo apt-get install -y openssh-server -qq &> /dev/null
# Remove this?
sudo apt-get install -y git -qq &> /dev/null
# Rapp-Text-To-Speech module depends on this.
sudo apt-get install -y espeak -qq &> /dev/null
# Rapp-Text-To-Speech module depends on this.
sudo apt-get install -y mbrola* -qq &> /dev/null
# Python package manager.
sudo apt-get install -y python-pip -qq &> /dev/null
# Node.js and Node.js package manager
sudo apt-get install -y npm nodejs -qq &> /dev/null
sudo ln -s /usr/bin/nodejs /usr/bin/node

# Install python yweather package
sudo pip install yweather &> /dev/null

# Grunt-Cli
sudo npm install -g grunt-cli &> /dev/null
sudo rm -rf ${HOME}/tmp &> /dev/null

# Enable Grunt shell auto-completion
append='eval "$(grunt --completion=bash)"'
grep -q "${append}" ~/.bashrc || echo -e          \
  "\n# Enable Grunt shell tab auto-completion\n${append}" \
  >> ~/.bashrc

# Load user's bash environment and flush output
source ~/.bashrc &> /dev/null
