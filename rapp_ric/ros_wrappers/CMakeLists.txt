cmake_minimum_required(VERSION 2.8.3)
project(ros_wrappers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  json_prolog
  rostest 
  rapp_platform_ros_communications
  message_generation
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    roscpp
    json_prolog
    rapp_platform_ros_communications
  INCLUDE_DIRS
    include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
)

###########
## Build ##
###########
add_subdirectory(src/mysql_wrapper)
add_subdirectory(src/knowrob_wrapper)
add_subdirectory(src/speech_rec_google)

