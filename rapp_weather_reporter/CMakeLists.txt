cmake_minimum_required(VERSION 2.8.3)
project(rapp_weather_reporter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rapp_utilities
  rapp_platform_ros_communications
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    std_msgs
    rapp_utilities
    rapp_platform_ros_communications
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  add_rostest(tests/yweather_reporter_current_test.launch)
  add_rostest(tests/yweather_reporter_forecast_test.launch)
endif()
