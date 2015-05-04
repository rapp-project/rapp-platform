#ifndef RAPP_QR_DETECTION_NODE
#define RAPP_QR_DETECTION_NODE

#include "ros/ros.h"

#include <rapp_platform_ros_communications/QrDetectionRosSrv.h>

#include <qr_detection/qr_detector.h>

class QrDetection
{
  public:

    // Default constructor
    QrDetection(void);

    bool qrDetectionCallback(
      rapp_platform_ros_communications::QrDetectionRosSrv::Request& req,
      rapp_platform_ros_communications::QrDetectionRosSrv::Response& res
      );

  private:
    // The ROS node handle
    ros::NodeHandle nh_;

    // The service server 
    ros::ServiceServer qrDetectionService_;
    
    // Topic nomeclarure
    std::string qrDetectionTopic_;
    
    QrDetector qr_detector_; 
};

#endif
