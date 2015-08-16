cmake_minimum_required(VERSION 2.8.3)
project(rapp_speech_detection_sphinx4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  rapp_platform_ros_communications
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS 
    roscpp 
    roslib
    rospy 
    std_msgs
    rapp_platform_ros_communications
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  # Unit
  catkin_add_nosetests(tests/unit/english_support_unit_tests.py)
  catkin_add_nosetests(tests/unit/greek_support_unit_tests.py)
  catkin_add_nosetests(tests/unit/limited_vocabulary_creator_unit_tests.py)

  # Functional
  add_rostest(tests/functional/configure_functional.launch)
  add_rostest(tests/functional/batch_functional.launch)

endif()

