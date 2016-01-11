
cmake_minimum_required(VERSION 2.8.3)
project(rapp_costmap_2d)

find_package(catkin REQUIRED
        COMPONENTS
              cmake_modules
              dynamic_reconfigure
              costmap_2d
              map_msgs
              message_filters
              nav_msgs
              pluginlib
              roscpp

        )
# Linux should be the default OS/Platform for RAPP::API
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  
  # using GCC/Linux
  message (STATUS "GCC/Linux" )
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  
  if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "Info: g++ version >= 4.7 using C++11")  
  elseif ()
    message (FATAL_ERROR "g++ version >= 4.7 is required")
  endif()
  
  # Do I need this ?
  #add_definitions(-DBOOST_THREAD_USE_LIB)
  
  # NOTE: gnu++11 enables GNU's violating options, whereas c++11 is strictly only the new version
  # static libstdc++ linkage solves issues in Gentoo/OpenNao where packages of newer gcc don't exist.
  # Ideally you should setup using export LD_LIBRARY_PATH, but this is a quick and dirty way of packaging librapp
  # -static-libstdc++
  # -stdlib=libc++ will link using an alternative to gcc std c++ library - WARNING not working on OpenNao
  # -static-libgcc will link statically GLIBC

  set(RAPP_CXX_FLAGS "-std=gnu++11 -Wall -fPIC -static-libstdc++")
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${RAPP_CXX_FLAGS})
  message(STATUS "CXX Flags: " ${CMAKE_CXX_FLAGS})
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  #set(CMAKE_EXE_LINKER_FLAGS "-lc++ -lcxxrt") # Set linker flags - WARNING alternative c++ libraries not present on OpenNao
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mtune=native -march=native") # -s ?
  set(BUILD_SHARED_LIBS OFF)
  message(STATUS "CXX Linker: " ${CMAKE_EXE_LINKER_FLAGS})

elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # using Clang
    message (STATUS "Clang/OSX" )
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (CLANG_VERSION_MAJOR VERSION_GREATER 3 OR CLANG_VERSION_MAJOR VERSION_EQUAL 3)
        message(STATUS "Info: Clang >= 3.0 using C++11")
    elseif ()
        message(FATAL_ERROR "Clang version >= 3.0 is required")
    endif()
        set(RAPP_CXX_FLAGS "-std=c++11 -Wall")
        set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${RAPP_CXX_FLAGS})
        message(STATUS "CXX Flags: " ${CMAKE_CXX_FLAGS})
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mtune=native -march=native")
        set(BUILD_SHARED_LIBS OFF)
        message(STATUS "CXX Linker: " ${CMAKE_EXE_LINKER_FLAGS})

# Intel Compiler May run on Linux - Maybe we should support it
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    message (FATAL_ERROR "unsupported compiler")

# We cannot support Microsoft Visual Studio - We cannot support Win32 platforms
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message (FATAL_ERROR "unsupported compiler")
endif()
find_package(costmap_2d REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${costmap_2d_INCLUDE_DIRS}
)

add_definitions(${costmap_2d_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/Costmap2D.cfg
  cfg/ObstaclePlugin.cfg
  cfg/GenericPlugin.cfg
  cfg/InflationPlugin.cfg
  cfg/VoxelPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES rapp_layers
    CATKIN_DEPENDS
        costmap_2d
              dynamic_reconfigure
              costmap_2d
              map_msgs
              message_filters
              nav_msgs
              pluginlib
              roscpp
)

add_library(rapp_layers SHARED
  plugins/rapp_static_layer.cpp
  src/observation_buffer.cpp
)
target_link_libraries(rapp_layers
  ${costmap_2d_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(rapp_layers costmap_2d_gencfg   rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS})

## Configure Tests
if (False)
if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)

  # Add the test folder to the include directories
  include_directories(test)

  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})

  # Create targets for test executables
  add_executable(costmap_tester EXCLUDE_FROM_ALL test/costmap_tester.cpp)
  add_dependencies(tests costmap_tester)
  target_link_libraries(costmap_tester costmap_2d ${GTEST_LIBRARIES})

  add_executable(footprint_tests EXCLUDE_FROM_ALL test/footprint_tests.cpp)
  add_dependencies(tests footprint_tests)
  target_link_libraries(footprint_tests costmap_2d ${GTEST_LIBRARIES})

  add_executable(obstacle_tests EXCLUDE_FROM_ALL test/obstacle_tests.cpp)
  add_dependencies(tests obstacle_tests)
  target_link_libraries(obstacle_tests costmap_2d layers ${GTEST_LIBRARIES})

  add_executable(static_tests EXCLUDE_FROM_ALL test/static_tests.cpp)
  add_dependencies(tests static_tests)
  target_link_libraries(static_tests costmap_2d layers ${GTEST_LIBRARIES})

  add_executable(inflation_tests EXCLUDE_FROM_ALL test/inflation_tests.cpp)
  add_dependencies(tests inflation_tests)
  target_link_libraries(inflation_tests costmap_2d layers ${GTEST_LIBRARIES})

  catkin_download_test_data(${PROJECT_NAME}_simple_driving_test_indexed.bag
    http://download.ros.org/data/costmap_2d/simple_driving_test_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 61168cff9425b11e093ea3a627c81c8d)
  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.025.pgm
    http://download.ros.org/data/costmap_2d/willow-full-0.025.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e66b17ee374f2d7657972efcb3e2e4f7)

  add_rostest(test/footprint_tests.launch)
  add_rostest(test/inflation_tests.launch)
  add_rostest(test/obstacle_tests.launch)
  add_rostest(test/simple_driving_test.xml)
  add_rostest(test/static_tests.launch)

  catkin_add_gtest(array_parser_test test/array_parser_test.cpp)
  target_link_libraries(array_parser_test costmap_2d)
endif()
endif()

install(TARGETS
    rapp_layers
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES rapp_costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
