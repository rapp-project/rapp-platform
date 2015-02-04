# Library for unit testing
add_library(knowrob_connector_lib
  knowrob_wrapper.cpp
  )
target_link_libraries(knowrob_connector_lib
  ${catkin_LIBRARIES}
  )
add_dependencies(knowrob_connector_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
  )

# RIS executable
add_executable(knowrob_wrapper_node
  knowrob_wrapper_node.cpp
  knowrob_wrapper_communications.cpp
  )
target_link_libraries(knowrob_wrapper_node
  knowrob_connector_lib
  ${catkin_LIBRARIES}
  )
add_dependencies(knowrob_wrapper_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
  )
