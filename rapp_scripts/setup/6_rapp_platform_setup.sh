#!/bin/bash 
sudo apt-get install -y libzbar-dev
source /opt/ros/indigo/setup.bash
source ~/rapp_platform/knowrob_catkin_ws/devel/setup.bash --extend
source ~/.bashrc
sudo ldconfig

echo -e "\e[1m\e[103m\e[31m [RAPP] Create Github folders \e[0m"
# Create folder for RAPP platform repo
cd ~/rapp_platform
mkdir rapp-platform-catkin-ws
cd rapp-platform-catkin-ws
mkdir src
cd src

echo -e "\e[1m\e[103m\e[31m [RAPP] Cloning the rapp-platform repo \e[0m"
# Clone the repository (public key should have been setup)
git clone git@github.com:rapp-project/rapp-platform.git
git clone git@github.com:rapp-project/rapp-api.git

catkin_init_workspace

echo "source ~/rapp_platform/rapp-platform-catkin-ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc

# catkin_make rapp-platform
cd $HOME/rapp_platform/rapp-platform-catkin-ws
catkin_make



