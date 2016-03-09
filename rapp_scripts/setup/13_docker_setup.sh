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

# Authors: Athanassios Kintsakis
# Contact: akintsakis@issel.ee.auth.gr
##

##
#  Docker installation.
#  For more information regarding Caffe itself visit:
#    https://www.docker.com/
##

sudo apt-get -y update
sudo apt-get -y install curl
curl -fsSL https://get.docker.com/ | sh
sudo usermod -aG docker $USER
sudo pip install docker-py

sudo ldconfig
source ~/.bashrc

docker pull ubuntu

echo -e "\e[1m\e[103m\e[31m [RAPP] Docker installation Finished \e[0m"
