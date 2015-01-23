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

add_subdirectory(inc)

find_package( Boost 1.49 COMPONENTS system REQUIRED )

add_executable ( rapp_server  main.cpp )

target_link_libraries( rapp_server ${Boost_SYSTEM_LIBRARY} )

set(CMAKE_COLOR_MAKEFILE on)
set(CMAKE_VERBOSE_MAKEFILE on)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

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
  
  # NOTE: gnu++11 enables GNU's violating options, whereas c++11 is strictly only the new version
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=gnu++11 -Wall")
  message(STATUS "CXX Flags: " ${CMAKE_CXX_FLAGS})
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mtune=native -march=native") # -s ?
  set(BUILD_SHARED_LIBS OFF)
  message(STATUS "CXX Linker: " ${CMAKE_EXE_LINKER_FLAGS})

# Intel Compiler May run on Linux - Maybe we should support it
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    message (FATAL_ERROR "unsupported compiler")

# We cannot support Microsoft Visual Studio - We cannot support Win32 platforms
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message (FATAL_ERROR "unsupported compiler")
endif()
