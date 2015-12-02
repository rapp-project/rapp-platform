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


RappPlatformPath="${HOME}/rapp_platform"
cmusphinxUrl="https://github.com/skerit/cmusphinx"
sphinxbaseUrl="https://github.com/cmusphinx/sphinxbase.git"

#download and compile sphinx4 extra libraries
echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Sphinx4 Libraries \e[0m"
sudo apt-get install -qq -y swig &> /dev/null
sudo apt-get install -qq -y autoconf &> /dev/null
sudo apt-get install -qq -y python-scipy &> /dev/null
sudo apt-get install -qq -y bison &> /dev/null
sudo apt-get install -qq -y sox &> /dev/null
sudo apt-get install -qq -y flac &> /dev/null

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Sphinx4 CMU_SPHINX\e[0m"
cd ${RappPlatformPath}
git clone ${cmusphinxUrl} &> /dev/null
cd cmusphinx/cmuclmtk
./autogen.sh &> /dev/null
make &> /dev/null
sudo make install &> /dev/null

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Sphinx4 Base \e[0m"
cd ${RappPlatformPath}
git clone ${sphinxbaseUrl}
cd sphinxbase
./autogen.sh &> /dev/null
make &> /dev/null
sudo make install &> /dev/null

echo -e "\e[1m\e[103m\e[31m [RAPP] Building Sphinx4.java wrapper \e[0m"
cd ${RappPlatformPath}/rapp-platform-catkin-ws/src/rapp-platform/rapp_speech_detection_sphinx4/src
bash buildJava.sh &> /dev/null
