## Library for unit testing
add_library(qr_detector_lib
  qr_detector.cpp
  )
target_link_libraries(qr_detector_lib
  ${catkin_LIBRARIES}
  zbar
  )

## ROS node executable
add_executable(qr_detection_ros_node 
  qr_detection.cpp
  qr_detection_node.cpp
)
target_link_libraries(qr_detection_ros_node
  qr_detector_lib
  ${catkin_LIBRARIES}
)
add_dependencies(qr_detection_ros_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Tests
if (CATKIN_ENABLE_TESTING)
  # unit tests
  catkin_add_gtest(qr_detection_unit_test 
    ../../test/qr_detection/unit_tests.cpp
    )
  target_link_libraries(qr_detection_unit_test 
    ${catkin_LIBRARIES}
    qr_detector_lib
    gtest_main
    )

  # functional tests
  add_rostest(../../test/qr_detection/functional_tests.launch)
endif()
