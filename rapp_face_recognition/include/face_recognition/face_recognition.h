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

#ifndef RAPP_FACE_RECOGNITION_NODE
#define RAPP_FACE_RECOGNITION_NODE

#include "ros/ros.h"

#include <rapp_platform_ros_communications/FaceRecognitionRosSrv.h>

#include <face_recognition/face_recognizer.h>


/**
 * @class FaceRecognition
 * @brief Class FaceRecognition uptakes the task of handling the ROS service callbacks
 */
class FaceRecognition
{
  public:

    /** 
     * @brief Default constructor
     */
    FaceRecognition(void);

    /**
     * @brief Serves the face recognition ROS service callback
     * @param req [rapp_platform_ros_communications::FaceRecognitionRosSrv::Request&] The ROS service request
     * @param res [rapp_platform_ros_communications::FaceRecognitionRosSrv::Response&] The ROS service response
     * @return bool - The success status of the call
     */
    bool faceRecognitionCallback(
      rapp_platform_ros_communications::FaceRecognitionRosSrv::Request& req,
      rapp_platform_ros_communications::FaceRecognitionRosSrv::Response& res
      );

  private:
    /**< The ROS node handle */
    ros::NodeHandle nh_;

    /**< The face recognition service server */
    ros::ServiceServer faceRecognitionService_;

    /**< Member variable holding the face recognition ROS service topic */
    std::string faceRecognitionTopic_;

    /**< Object of type FaceRecognizer */
    FaceRecognizer face_recognizer_;
};

#endif // RAPP_FACE_RECOGNITION_NODE
