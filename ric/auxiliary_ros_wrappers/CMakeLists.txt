cmake_minimum_required(VERSION 2.8.3)
project(auxiliary_ros_wrappers)

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
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(auxiliary_ros_wrappers
#   src/${PROJECT_NAME}/auxiliary_ros_wrappers.cpp
# )

add_subdirectory(src/knowrob_wrapper)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_auxiliary_ros_wrappers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#if (CATKIN_ENABLE_TESTING)
  #catkin_add_nosetests(test/mysql_wrapper/t1.py)
  #add_rostest(test/mysql_wrapper/db_test.launch)
#endif()
if (CATKIN_ENABLE_TESTING)
  add_rostest(test/speech_to_text/functional_tests.launch)
endif()
