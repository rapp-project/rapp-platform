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

  Author: Athanassios Kintsakis
  contact: akintsakis@issel.ee.auth.gr

******************************************************************************/

#include <qr_detection/qr_detection.h>

QrDetection::QrDetection(void)
{
  if(!nh_.getParam("/rapp_qr_detection_detect_qrs_topic", qrDetectionTopic_))
  {
    ROS_ERROR("Qr detection topic param does not exist");
  }
  // Creating the service server concerning the face detection functionality
  qrDetectionService_ = nh_.advertiseService(qrDetectionTopic_,
    &QrDetection::qrDetectionCallback, this);
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
