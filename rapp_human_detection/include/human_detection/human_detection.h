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

#ifndef RAPP_HUMAN_DETECTION_NODE
#define RAPP_HUMAN_DETECTION_NODE

#include "ros/ros.h"

#include <rapp_platform_ros_communications/HumanDetectionRosSrv.h>

#include <human_detection/human_detector.h>

/**
 * @class HumanDetection
 * @brief Class HumanDetection uptakes the task of handling the ROS service callbacks
 */
class HumanDetection
{
  public:

    /** 
     * @brief Default constructor
     */
    HumanDetection(void);

    /**
     * @brief Serves the human detection ROS service callback
     * @param req [rapp_platform_ros_communications::HumanDetectionRosSrv::Request&] The ROS service request
     * @param res [rapp_platform_ros_communications::HumanDetectionRosSrv::Response&] The ROS service response
     * @return bool - The success status of the call
     */
    bool humanDetectionCallback(
      rapp_platform_ros_communications::HumanDetectionRosSrv::Request& req,
      rapp_platform_ros_communications::HumanDetectionRosSrv::Response& res
      );

  private:
    /**< The ROS node handle */
    ros::NodeHandle nh_;

    /**< The human detection service server */
    ros::ServiceServer humanDetectionService_;

    /**< Member variable holding the human detection ROS service topic */
    std::string humanDetectionTopic_;

    /**< Object of type HumanDetector */
    HumanDetector human_detector_;
};

#endif // RAPP_HUMAN_DETECTION_NODE
