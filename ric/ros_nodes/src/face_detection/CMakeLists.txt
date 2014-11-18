## Library for unit testing
add_library(face_detector_lib
  face_detector.cpp
  )
target_link_libraries(face_detector_lib
  ${catkin_LIBRARIES}
  )

## ROS node executable
add_executable(face_detection_ros_node 
  face_detection.cpp
  face_detection_node.cpp
)
target_link_libraries(face_detection_ros_node
  face_detector_lib
  ${catkin_LIBRARIES}
)
add_dependencies(face_detection_ros_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Tests
if (CATKIN_ENABLE_TESTING)
  # unit tests
  #catkin_add_gtest(face_detection_unit_test 
    #../../test/face_detection/unit_tests.cpp
    #)
  #target_link_libraries(face_detection_unit_test 
    #${catkin_LIBRARIES}
    #face_detector_lib
    #gtest_main
    #)

  # functional tests
  #add_rostest(../../test/face_detection/functional_tests.launch)
endif()
