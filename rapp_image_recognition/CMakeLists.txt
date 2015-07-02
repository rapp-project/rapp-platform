cmake_minimum_required(VERSION 2.8.3)
project(rapp_image_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
)

add_service_files(
  FILES
  FindObjects.srv
  UploadImage.srv
  LightCheck.srv
)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(find_objects_node src/find_objects_node.cpp src/find_objects.cpp)
target_link_libraries(find_objects_node ${catkin_LIBRARIES})
add_dependencies(find_objects_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(upload_image_node src/upload_image_node.cpp)
target_link_libraries(upload_image_node ${catkin_LIBRARIES})
add_dependencies(upload_image_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(light_check_node src/light_check_node.cpp src/light_check.cpp)
target_link_libraries(light_check_node ${catkin_LIBRARIES})
add_dependencies(light_check_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS find_objects_node upload_image_node light_check_node
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
