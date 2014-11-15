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
  std::vector<cv::Rect> faces = face_detector_.findFaces(req.imageFilename);
  
  for(unsigned int i = 0 ; i < faces.size() ; i++)
  {
    geometry_msgs::PointStamped up_left_corner;
    geometry_msgs::PointStamped down_right_corner;

    up_left_corner.point.x = faces[i].x;
    up_left_corner.point.y = faces[i].y;

    down_right_corner.point.x = faces[i].x + faces[i].width;
    down_right_corner.point.y = faces[i].y + faces[i].height;
    
    res.faces_up_left.push_back(up_left_corner);
    res.faces_down_right.push_back(down_right_corner);
  }

  return true;
}
