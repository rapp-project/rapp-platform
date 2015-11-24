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

# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# Contact: klpanagi@gmail.com, etsardou@iti.gr
##

##
#  Upgrade system packages/libraries.
##

echo -e "\e[1m\e[103m\e[31m [RAPP] Initiating source lists \e[0m"
# Refresh source lists
sudo apt-get update

echo -e "\e[1m\e[103m\e[31m [RAPP] Upgrading packages \e[0m"
# Upgrades the current packages
sudo apt-get upgrade -y


