#include <fstream>
#include <string>
#include <vector>

#include <rapp_server/serviceHandler/serviceHandler.hpp>

#include <rapp_platform_ros_communications/SpeechToTextSrv.h>

#include "ros/ros.h"
#include "ros/package.h"

// Number to string - easy and nasty
#define TOSTR( x ) static_cast< std::ostringstream & >( \
  ( std::ostringstream() << std::dec << x ) ).str()

