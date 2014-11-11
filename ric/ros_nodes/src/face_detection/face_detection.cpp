#include <face_detection/face_detection.h>

FaceDetection::FaceDetection(void)
{
  faceDetectionTopic_ = "ric/face_detection_service";

  // Creating the service server concerning the face detection functionality
  faceDetectionService_ = nh_.advertiseService(faceDetectionTopic_, 
    &FaceDetection::faceDetectionCallback, this);
}

bool FaceDetection::faceDetectionCallback(
  rapp_platform_ros_communications::FaceDetectionRosSrv::Request& req,
  rapp_platform_ros_communications::FaceDetectionRosSrv::Response& res)
{
  ROS_ERROR("Service called");
  return true;
}

