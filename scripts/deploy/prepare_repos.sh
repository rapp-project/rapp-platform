#!/bin/bash

# Synchronize Knowrob
cd ~/knowrob_catkin_ws
rm -rf build/ devel/
cd src/knowrob
git fetch
git checkout indigo-devel
git reset --hard origin/indigo-devel
cd ~/knowrob_catkin_ws
catkin_make

# Synchronize RAPP platform repos
cd ~/rapp_platform_catkin_ws
rm -rf build/ devel/
cd src/rapp-platform
git fetch
git checkout master
git reset --hard origin/master
cd ~/rapp_platform_catkin_ws/src/rapp-platform-supplementary-material
git fetch
git checkout master
git reset --hard origin/master
cd ~/rapp_platform_catkin_ws
catkin_make
