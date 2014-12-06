#include <qr_detection/qr_detection.h>

QrDetection::QrDetection(void)
{
  qrDetectionTopic_ = "ric/ros_nodes/qr_detection_service";

  // Creating the service server concerning the face detection functionality
  qrDetectionService_ = nh_.advertiseService(qrDetectionTopic_, 
    &QrDetection::qrDetectionCallback, this);
}

bool QrDetection::qrDetectionCallback(
  rapp_platform_ros_communications::QrDetectionRosSrv::Request& req,
  rapp_platform_ros_communications::QrDetectionRosSrv::Response& res)
{
  std::vector<cv::Rect> qr_points;
  std::vector<std::string> qr_messages;
  qr_detector_.findQrs(req.imageFilename, qr_points, qr_messages);
  //for(unsigned int i = 0 ; i < faces.size() ; i++)
  //{
    //geometry_msgs::PointStamped up_left_corner;
    //geometry_msgs::PointStamped down_right_corner;

    //up_left_corner.point.x = faces[i].x;
    //up_left_corner.point.y = faces[i].y;

    //down_right_corner.point.x = faces[i].x + faces[i].width;
    //down_right_corner.point.y = faces[i].y + faces[i].height;
    
    //res.faces_up_left.push_back(up_left_corner);
    //res.faces_down_right.push_back(down_right_corner);
  //}

  return true;
}
