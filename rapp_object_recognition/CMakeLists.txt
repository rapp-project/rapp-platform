cmake_minimum_required(VERSION 2.8.3)
project(rapp_object_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  rapp_platform_ros_communications
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rostest
    rapp_platform_ros_communications
    cv_bridge
  INCLUDE_DIRS
    include
)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(BASE_NAME find_objects)

## Library for unit testing
add_library(${BASE_NAME}_lib
  src/${BASE_NAME}.cpp
)

target_link_libraries(${BASE_NAME}_lib
  ${catkin_LIBRARIES}
)

add_dependencies(${BASE_NAME}_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## ROS node executable
add_executable(${BASE_NAME}_node
  src/${BASE_NAME}_node.cpp
)

target_link_libraries(${BASE_NAME}_node
  ${BASE_NAME}_lib
  ${catkin_LIBRARIES}
)

add_dependencies(${BASE_NAME}_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS find_objects_lib find_objects_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch 
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  # unit tests
  catkin_add_gtest(${BASE_NAME}_unit_test
    tests/${BASE_NAME}/unit_tests.cpp
  )
  target_link_libraries(${BASE_NAME}_unit_test
    ${catkin_LIBRARIES}
    ${BASE_NAME}_lib
    gtest_main
  )

  # functional tests
  #add_rostest(tests/${BASE_NAME}/functional_tests.launch)
endif()
