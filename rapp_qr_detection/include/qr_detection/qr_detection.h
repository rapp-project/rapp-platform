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

#ifndef RAPP_QR_DETECTION_NODE
#define RAPP_QR_DETECTION_NODE

#include "ros/ros.h"

#include <rapp_platform_ros_communications/QrDetectionRosSrv.h>

#include <qr_detection/qr_detector.h>

/**
 * @class QrDetection
 * @brief Uptakes the task of setting up the ROS service callbacks towards qr detection
 */
class QrDetection
{
  public:

    /** 
     * @brief Default constructor 
     */
    QrDetection(void);

    /**
     * @brief The qr detection ROS service callback
     * @param req [rapp_platform_ros_communications::QrDetectionRosSrv::Request&] The service request
     * @param res [rapp_platform_ros_communications::QrDetectionRosSrv::Response&] The service response
     * @return bool - The success status of the call
     */
    bool qrDetectionCallback(
      rapp_platform_ros_communications::QrDetectionRosSrv::Request& req,
      rapp_platform_ros_communications::QrDetectionRosSrv::Response& res
      );

  private:
    /**< The ROS node handle */
    ros::NodeHandle nh_;

    /**< The service server */
    ros::ServiceServer qrDetectionService_;

    /**< Topic nomeclarure. Holds the qr detection ROS service topic URI */
    std::string qrDetectionTopic_;

    /**< Object of QrDetection type */
    QrDetector qr_detector_;
};

#endif // RAPP_QR_DETECTION_NODE
