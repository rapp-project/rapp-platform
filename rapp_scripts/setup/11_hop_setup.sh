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

# Authors: Konstantinos Panyiotou
# Contact: klpanagi@gmail.com
##


RappPlatformPath="${HOME}/rapp_platform"
BiglooIndex="bigloo4.2c-beta08Dec15"
BiglooUrl="ftp://ftp-sop.inria.fr/indes/fp/Bigloo/${BiglooIndex}.tar.gz"
HopRepoUrl="https://github.com/manuel-serrano/hop.git"
HopCommitIndex="7673c6e318425e880a05818626f689229699e157"

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP by repos \e[0m"

# Keep current directory
curr=$(pwd)
#sudo sh -c 'echo "deb ftp://ftp-sop.inria.fr/indes/rapp/UBUNTU lts hop" >> \
#  /etc/apt/sources.list'

sudo apt-get install libunistring-dev &> /dev/null

# Bigloo installation
cd ${RappPlatformPath}
mkdir hop-bigloo
cd hop-bigloo
wget ${BiglooUrl} &> /dev/null

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Bigloo \e[0m"
tar -zxf "${BiglooIndex}.tar.gz" &> /dev/null
cd bigloo4.2c
./configure &> /dev/null
make &> /dev/null
make test &> /dev/null
make compile-bee &> /dev/null
sudo make install &> /dev/null
sudo make install-bee &> /dev/null

# HOP installation
echo -e "\e[1m\e[103m\e[31m [RAPP] Installing HOP \e[0m"
cd ..
git clone ${HopRepoUrl} &> /dev/null
cd hop
git checkout ${HopCommitIndex} &> /dev/null
./configure &> /dev/null
make &> /dev/null
sudo make install &> /dev/null
make doc &> /dev/null
sudo make install &> /dev/null

echo -e "\e[1m\e[103m\e[31m [RAPP] Finished HOP/Bigloo \e[0m"

# Initialize HOP with users
#mkdir -p $HOME/.config/hop
#cd $curr/hop_init
#cp  $HOME/.config/hop/
