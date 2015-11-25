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

#ifndef RAPP_FACE_DETECTION_NODE
#define RAPP_FACE_DETECTION_NODE

#include "ros/ros.h"

#include <rapp_platform_ros_communications/FaceDetectionRosSrv.h>

#include <face_detection/face_detector.h>

/**
 * @class FaceDetection
 * @brief Class FaceDetection uptakes the task of handling the ROS service callbacks
 */
class FaceDetection
{
  public:

    /** 
     * @brief Default constructor
     */
    FaceDetection(void);

    /**
     * @brief Serves the face detection ROS service callback
     * @param req [rapp_platform_ros_communications::FaceDetectionRosSrv::Request&] The ROS service request
     * @param res [rapp_platform_ros_communications::FaceDetectionRosSrv::Response&] The ROS service response
     * @return bool - The success status of the call
     */
    bool faceDetectionCallback(
      rapp_platform_ros_communications::FaceDetectionRosSrv::Request& req,
      rapp_platform_ros_communications::FaceDetectionRosSrv::Response& res
      );

  private:
    /**< The ROS node handle */
    ros::NodeHandle nh_;

    /**< The face detection service server */
    ros::ServiceServer faceDetectionService_;

    /**< Member variable holding the face detection ROS service topic */
    std::string faceDetectionTopic_;

    /**< Object of type FaceDetector */
    FaceDetector face_detector_;
};

#endif // RAPP_FACE_DETECTION_NODE
