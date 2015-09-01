#!/bin/bash 

# Download and install KnowRob
echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Knowrob \e[0m"
sudo apt-get install -y swi-prolog swi-prolog-java
sudo apt-get install -y ros-indigo-json-prolog-msgs
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-indigo-data-vis-msgs libjson-glib-dev
sudo apt-get install -y ros-indigo-rosjava-build-tools
echo "export SWI_HOME_DIR=/usr/lib/swi-prolog" >> ~/.bashrc
sudo ldconfig
source ~/.bashrc
cd ~/
mkdir -p rapp_platform/knowrob_catkin_ws
cd rapp_platform/knowrob_catkin_ws
mkdir src
cd src
catkin_init_workspace

git clone https://github.com/rapp-project/knowrob.git
cd knowrob
git checkout indigo-devel
cd ../../
catkin_make
echo "source ~/rapp_platform/knowrob_catkin_ws/devel/setup.bash --extend" >> ~/.bashrc
sudo ldconfig
source ~/.bashrc

