#ifndef ROS_SERVICE_INCLUDES_HPP
#define ROS_SERVICE_INCLUDES_HPP

#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "ros/package.h"

// Number to string - easy and nasty
#define TOSTR( x ) static_cast< std::ostringstream & >( \
  ( std::ostringstream() << std::dec << x ) ).str()

#endif // ROS_SERVICE_INCLUDES_HPP
