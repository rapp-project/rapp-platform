#ifndef RAPP_FACE_DETECTION_NODE
#define RAPP_FACE_DETECTION_NODE

#include "ros/ros.h"

#include <rapp_platform_ros_communications/FaceDetectionRosSrv.h>

#include <face_detection/face_detector.h>

class FaceDetection
{
  public:

    // Default constructor
    FaceDetection(void);

    bool faceDetectionCallback(
      rapp_platform_ros_communications::FaceDetectionRosSrv::Request& req,
      rapp_platform_ros_communications::FaceDetectionRosSrv::Response& res
      );

  private:
    // The ROS node handle
    ros::NodeHandle nh_;

    // The service server 
    ros::ServiceServer faceDetectionService_;
    std::vector<ros::ServiceServer> faceDetectionThreadServices_;

    // Topic nomeclarure
    std::string faceDetectionTopic_;
    
    FaceDetector face_detector_; 
};

#endif
