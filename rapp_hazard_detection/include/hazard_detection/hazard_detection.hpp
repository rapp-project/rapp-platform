/******************************************************************************
Copyright 2015 RAPP

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

******************************************************************************/

#ifndef RAPP_HAZARD_DETECTION
#define RAPP_HAZARD_DETECTION

#include "ros/ros.h"

#include <rapp_platform_ros_communications/LightCheckRosSrv.h>
#include <rapp_platform_ros_communications/DoorCheckRosSrv.h>

#include <hazard_detection/light_check.hpp>
#include <hazard_detection/door_check.hpp>

/**
 * @class HazardDetection
 * @brief Class HazardDetection uptakes the task of handling the ROS service callbacks
 */
class HazardDetection
{
  public:

    /** 
     * @brief Default constructor
     */
    HazardDetection(void);

    /**
     * @brief Serves the light check ROS service callback
     * @param req [rapp_platform_ros_communications::LightCheckRosSrv::Request&] The ROS service request
     * @param res [rapp_platform_ros_communications::LightCheckRosSrv::Response&] The ROS service response
     * @return bool - The success status of the call
     */
    bool lightCheckCallback(
      rapp_platform_ros_communications::LightCheckRosSrv::Request& req,
      rapp_platform_ros_communications::LightCheckRosSrv::Response& res
      );

    /**
     * @brief Serves the door check ROS service callback
     * @param req [rapp_platform_ros_communications::DoorCheckRosSrv::Request&] The ROS service request
     * @param res [rapp_platform_ros_communications::DoorCheckRosSrv::Response&] The ROS service response
     * @return bool - The success status of the call
     */
    bool doorCheckCallback(
      rapp_platform_ros_communications::DoorCheckRosSrv::Request& req,
      rapp_platform_ros_communications::DoorCheckRosSrv::Response& res
      );

  private:
    /**< The ROS node handle */
    ros::NodeHandle nh_;

    /**< The light check service server */
    ros::ServiceServer lightCheckService_;

    /**< Member variable holding the light check ROS service name */
    std::string lightCheckTopic_;

    /**< Light check implementation*/
    LightCheck light_check;

    /**< The door check service server */
    ros::ServiceServer doorCheckService_;

    /**< Member variable holding the door check ROS service name */
    std::string doorCheckTopic_;
};

#endif // RAPP_HAZARD_DETECTION
