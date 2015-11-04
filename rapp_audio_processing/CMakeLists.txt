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
  # Unit
  catkin_add_nosetests(tests/unit/set_noise_profile_unit_tests.py)
  catkin_add_nosetests(tests/unit/utilities_unit_tests.py)
  catkin_add_nosetests(tests/unit/detect_silence_unit_tests.py)
  catkin_add_nosetests(tests/unit/denoise_unit_tests.py)
  catkin_add_nosetests(tests/unit/energy_denoise_unit_tests.py)
  catkin_add_nosetests(tests/unit/transform_audio_unit_tests.py)
  # Functional
  add_rostest(tests/functional/set_noise_profile_functional.launch)
endif()
