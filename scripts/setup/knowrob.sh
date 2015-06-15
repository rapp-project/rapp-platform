#!/bin/bash 

# Download and install KnowRob
sudo apt-get install -y swi-prolog swi-prolog-java
sudo apt-get install ros-indigo-json-prolog-msgs
echo "export SWI_HOME_DIR=/usr/lib/swi-prolog" >> ~/.bashrc
source ~/.bashrc
cd ~/
mkdir knowrob_catkin_ws
cd knowrob_catkin_ws
mkdir src
cd src
catkin_init_workspace
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-indigo-data-vis-msgs libjson-glib-dev
#git clone https://github.com/knowrob/knowrob.git
git clone https://github.com/rapp-project/knowrob.git
cd knowrob
git checkout indigo-devel
cd ../../
catkin_make
echo "source ~/knowrob_catkin_ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc
