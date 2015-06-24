#!/bin/bash

#------- General bash
export PATH=$PATH:/opt/bigloo/bin:/opt/hop/bin

#------- ROS related
source /opt/ros/indigo/setup.bash
source /home/etsardou/knowrob_catkin_ws/devel/setup.bash --extend
source /home/etsardou/rosjava/devel/setup.bash --extend
source /home/etsardou/rapp_platform_catkin_ws/devel/setup.bash --extend

#------- KnowRob related
export JAVA_HOME=/usr/lib/jvm/default-java
export SWI_HOME_DIR=/usr/lib/swi-prolog
export LD_LIBRARY_PATH=/usr/lib/jvm/default-java/jre/lib/amd64:/usr/lib/jvm/default-java/jre/lib/amd64/server:$LD_LIBRARY_PATH

HOP_SERVICES_PATH=/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform/hop_services/services

hop -p 9001 $HOP_SERVICES_PATH/runWorkers.js
