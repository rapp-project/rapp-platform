#include <face_detection/face_detection.h>

FaceDetection::FaceDetection(void)
{
  if(!nh_.getParam("/rapp_face_detection_detect_faces_topic", faceDetectionTopic_))
  {
    ROS_ERROR("Face detection topic param does not exist");
  }
  // Creating the service server concerning the face detection functionality
  faceDetectionService_ = nh_.advertiseService(faceDetectionTopic_, 
    &FaceDetection::faceDetectionCallback, this);
}

bool FaceDetection::faceDetectionCallback(
  rapp_platform_ros_communications::FaceDetectionRosSrv::Request& req,
  rapp_platform_ros_communications::FaceDetectionRosSrv::Response& res)
{
  std::vector<unsigned int [4]> history;
  std::vector<cv::Rect> faces = face_detector_.findFaces(req.imageFilename);
  for(unsigned int i = 0 ; i < faces.size() ; i++)
  {
  
    unsigned int temp[4];
    temp[0] = faces[i].x;
    temp[1] = faces[i].y;
    temp[2] = faces[i].x + faces[i].width;
    temp[3] = faces[i].y + faces[i].height;

    bool duplicate = false;
    for(unsigned int j = 0 ; j < history.size() ; j++)
    {
      if(temp[0] == history[j][0] && temp[1] == history[j][1] &&
         temp[2] == history[j][2] && temp[3] == history[j][3])
      {
        duplicate = true;
        break;    
      }
    }
    if(duplicate)
    {
      continue;
    }

    geometry_msgs::PointStamped up_left_corner;
    geometry_msgs::PointStamped down_right_corner;

    up_left_corner.point.x = temp[0];
    up_left_corner.point.y = temp[1];
    down_right_corner.point.x = temp[2];
    down_right_corner.point.y = temp[3];
   
    res.faces_up_left.push_back(up_left_corner);
    res.faces_down_right.push_back(down_right_corner);
  }

  return true;
}
