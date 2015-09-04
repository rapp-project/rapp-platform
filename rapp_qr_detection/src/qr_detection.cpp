#include <qr_detection/qr_detection.h>

QrDetection::QrDetection(void)
{
  if(!nh_.getParam("/rapp_qr_detection_detect_qrs_topic", qrDetectionTopic_))
  {
    ROS_ERROR("Qr detection topic param does not exist");
  }
  int qr_detection_threads = 0;
  if(!nh_.getParam("/rapp_qr_detection_threads", qr_detection_threads))
  {
    ROS_ERROR("Qr detection threads param not existent");
  }
  // Creating the service server concerning the face detection functionality
  qrDetectionService_ = nh_.advertiseService(qrDetectionTopic_, 
    &QrDetection::qrDetectionCallback, this);

  char init = '0';
  for(unsigned int i = 0 ; i < (unsigned int)qr_detection_threads ; i++)
  {
    qrDetectionServiceThreads_.push_back(
      nh_.advertiseService(
        qrDetectionTopic_ + std::string("_") + char(init + i),
        &QrDetection::qrDetectionCallback, 
        this
      )
    );  
  }

}

bool QrDetection::qrDetectionCallback(
  rapp_platform_ros_communications::QrDetectionRosSrv::Request& req,
  rapp_platform_ros_communications::QrDetectionRosSrv::Response& res)
{
  std::vector<cv::Point> qr_points;
  std::vector<std::string> qr_messages;
  qr_detector_.findQrs(req.imageFilename, qr_points, qr_messages);
  
  for(unsigned int i = 0 ; i < qr_points.size() ; i++)
  {
    geometry_msgs::PointStamped qr_center;

    qr_center.point.x = qr_points[i].x;
    qr_center.point.y = qr_points[i].y;

    res.qr_messages.push_back(qr_messages[i]);
    res.qr_centers.push_back(qr_center);
  }

  return true;
}
