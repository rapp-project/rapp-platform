#!/bin/bash 

# Download and install rosjava for KnowRob to work
sudo apt-get install python-wstool
mkdir -p ~/rosjava
wstool init -j4 ~/rosjava/src rosjava.rosinstall # Check here for path!
source /opt/ros/indigo/setup.bash
cd ~/rosjava
rosdep update
rosdep install --from-paths src -i -y
catkin_make
echo "source ~/rosjava/devel/setup.bash --extend" >> ~/.bashrc
echo "export JAVA_HOME=/usr/lib/jvm/default-java" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
