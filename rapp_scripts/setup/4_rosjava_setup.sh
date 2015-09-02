#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Rosjava \e[0m"

# Download and install rosjava for KnowRob to work
sudo apt-get install -y python-wstool
mkdir -p $HOME/rapp_platform/rosjava
wstool init -j4 $HOME/rapp_platform/rosjava/src rosjava.rosinstall # Check here for path!
source /opt/ros/indigo/setup.bash
cd $HOME/rapp_platform/rosjava
rosdep update
rosdep install --from-paths src -i -y
source ~/.bashrc
catkin_make
echo "source ~/rapp_platform/rosjava/devel/setup.bash --extend" >> ~/.bashrc
echo "export JAVA_HOME=/usr/lib/jvm/default-java" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc

