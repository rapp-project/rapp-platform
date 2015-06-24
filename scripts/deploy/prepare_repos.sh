#!/bin/bash

# Synchronize Knowrob
echo -e "\e[1m\e[103m\e[31m [RAPP] Synchronizing KnowRob \e[0m"
cd ~/knowrob_catkin_ws
rm -rf build/ devel/
cd src/knowrob
echo -e "\e[1m\e[103m\e[31m [RAPP] Checking out indigo-devel \e[0m"
git fetch
git checkout indigo-devel
git reset --hard origin/indigo-devel
cd ~/knowrob_catkin_ws
echo -e "\e[1m\e[103m\e[31m [RAPP] Building KnowRob \e[0m"
catkin_make

# Synchronize RAPP platform repos
echo -e "\e[1m\e[103m\e[31m [RAPP] Synchronizing RAPP Platform repositories \e[0m"
cd ~/rapp_platform_catkin_ws
rm -rf build/ devel/
cd src/rapp-platform
echo -e "\e[1m\e[103m\e[31m [RAPP] Checking out master for rapp-platform \e[0m"
git fetch
git checkout master
git reset --hard origin/master
cd ~/rapp_platform_catkin_ws/src/rapp-platform-supplementary-material
echo -e "\e[1m\e[103m\e[31m [RAPP] Checking out master for rapp-platform-supplementary-material \e[0m"
git fetch
git checkout master
git reset --hard origin/master
cd ~/rapp_platform_catkin_ws
echo -e "\e[1m\e[103m\e[31m [RAPP] Building RAPP Platform repositories \e[0m"
catkin_make
