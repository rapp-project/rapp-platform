cmake_minimum_required(VERSION 2.8.3)
project(rapp_audio_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
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
  catkin_add_nosetests(tests/unit/set_noise_profile_unit_tests.py)
  catkin_add_nosetests(tests/unit/cleanup_unit_tests.py)
  catkin_add_nosetests(tests/unit/detect_silence_unit_tests.py)
  catkin_add_nosetests(tests/unit/denoise_unit_tests.py)
  catkin_add_nosetests(tests/unit/energy_denoise_unit_tests.py)
endif()
