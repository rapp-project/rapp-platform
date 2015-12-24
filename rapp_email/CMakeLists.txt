cmake_minimum_required(VERSION 2.8.3)
project(rapp_email)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rapp_utilities
  rapp_platform_ros_communications
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  rospy
  rapp_utilities
  rapp_platform_ros_communications
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()


if (CATKIN_ENABLE_TESTING)
  add_rostest(tests/email_receiver_unit_tests.launch)
  add_rostest(tests/email_sender_unit_tests.launch)
endif()
