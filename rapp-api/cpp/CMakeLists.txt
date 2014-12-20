PROJECT(rapp)

set (rapp_VERSION_MAJOR 0)
set (rapp_VERSION_MINOR 1)

cmake_minimum_required(VERSION 2.8)

set(LIBRARY_PATH ${LIBRARY_PATH}
		      /lib
		      /usr/lib
		      /usr/lib64
		      /usr/local/lib
		      /usr/local/lib64
		      /usr/lib/x86_64-linux-gnu)

set(INCLUDE_HEADERS ${INCLUDE_HEADERS} /usr/include/)

include_directories(SYSTEM ${INCLUDE_HEADERS})

add_subdirectory(src)

# Need Boost System for ASIO
find_package( Boost 1.49 COMPONENTS system REQUIRED )

enable_testing()
# Test 
add_executable ( async_client test.cpp
                              src/asio_service_client.cpp)
                       
add_test ( TestAsyncClient async_client )

# Build Libraries (both static and shared)
set (SOURCES ${SOURCES} src/asio_service_client.cpp)

add_library (lrapp OBJECT ${SOURCES})

# There must be a better way to set the library version name
add_library (rapp.0.1-static STATIC $<TARGET_OBJECTS:lrapp>)
add_library (rapp.0.1 SHARED $<TARGET_OBJECTS:lrapp>)


target_link_libraries(rapp.0.1-static ${Boost_SYSTEM_LIBRARY} )
target_link_libraries(rapp.0.1 ${Boost_SYSTEM_LIBRARY} )
target_link_libraries(async_client ${Boost_SYSTEM_LIBRARY} )


set(CMAKE_COLOR_MAKEFILE on)
set(CMAKE_VERBOSE_MAKEFILE on)


#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug) #set only when coding/testing

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  
  # using Clang
  message (STATUS "Clang/OSX" )
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  
  if (CLANG_VERSION_MAJOR VERSION_GREATER 3 OR CLANG_VERSION_MAJOR VERSION_EQUAL 3)
    message(STATUS "Info: Clang >= 3.0 using C++11")
  
  elseif ()
    message(FATAL_ERROR "Clang version >= 3.0 is required")
  endif()
  
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall")
  message(STATUS "CXX Flags: " ${CMAKE_CXX_FLAGS})
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mtune=native -march=native")
  set(BUILD_SHARED_LIBS OFF)
  message(STATUS "CXX Linker: " ${CMAKE_EXE_LINKER_FLAGS})

elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  
  # using GCC/Linux
  message (STATUS "GCC/Linux" )
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  
  if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "Info: g++ version >= 4.7 using C++11")
  
  elseif ()
    message (FATAL_ERROR "g++ version >= 4.7 is required")
  endif()
  
  # NOTE: gnu++11 enables GNU's violating options, whereas c++11 is strictly only the new version
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=gnu++11 -Wall -fPIC")
  message(STATUS "CXX Flags: " ${CMAKE_CXX_FLAGS})
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mtune=native -march=native") # -s ?
  set(BUILD_SHARED_LIBS OFF)
  message(STATUS "CXX Linker: " ${CMAKE_EXE_LINKER_FLAGS})

elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    message (FATAL_ERROR "unsupported compiler")
    
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message (FATAL_ERROR "unsupported compiler")
endif()

#set(MACOSX_RPATH ON)

# Install stuff
install (TARGETS rapp.0.1-static  DESTINATION /usr/local/lib/)
install (TARGETS rapp.0.1 DESTINATION /usr/local/lib/)
install (DIRECTORY src/ DESTINATION /usr/local/include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.ihh")

#install (FILES includes/cgpp DESTINATION /usr/local/include/${PROJECT_NAME}) - NOTE: Use to install a global include header
