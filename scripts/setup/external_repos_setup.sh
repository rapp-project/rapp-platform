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

# Download and install KnowRob
sudo apt-get install -y swi-prolog swi-prolog-java
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
git clone https://github.com/knowrob/knowrob.git
cd knowrob
git checkout indigo-devel
cd ../../
catkin_make
echo "source ~/knowrob_catkin_ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc

