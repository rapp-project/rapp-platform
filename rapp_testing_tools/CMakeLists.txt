cmake_minimum_required(VERSION 2.8.3)
project(rapp_testing_tools)
set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  scripts/default_tests
)

catkin_install_python(
  PROGRAMS scripts/rapp_run_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install python modules under this package
catkin_python_setup()
