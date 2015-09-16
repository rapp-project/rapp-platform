cmake_minimum_required(VERSION 2.8.3)
project(rapp_mysql_wrapper)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rostest 
  rapp_platform_ros_communications
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
    rospy
    std_msgs
    rostest
    rapp_platform_ros_communications
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}  
)

###########
## Build ##
###########
if (CATKIN_ENABLE_TESTING)
  #catkin_add_nosetests(test/mysql_wrapper/t1.py)
  #add_rostest(test/mysql_wrapper/db_test.launch)
endif()
