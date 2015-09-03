#include <face_detection/face_detection.h>

FaceDetection::FaceDetection(void)
{
  int threads;
  if(!nh_.getParam("/rapp_face_detection_detect_faces_topic", faceDetectionTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }
  if(!nh_.getParam("/rapp_face_detection_threads", threads))
  {
    ROS_ERROR("Face detection threads param not found");
    threads = 0;
  }
  else if(threads < 0)
  {
    threads = 0;
  }
  // Creating the service server concerning the face detection functionality
  faceDetectionService_ = nh_.advertiseService(faceDetectionTopic_, 
    &FaceDetection::faceDetectionCallback, this);
  char init = '0';
  for(unsigned int i = 0 ; i < (unsigned int)threads ; i++)
  {
    faceDetectionThreadServices_.push_back(
      nh_.advertiseService(
        faceDetectionTopic_ + std::string("_") + char(init + i),
        &FaceDetection::faceDetectionCallback, 
        this
      )
    );  
  }
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
