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

#include <human_detection/human_detection.h>


/** 
 * @brief Default constructor. Performs initializations.
 */
HumanDetection::HumanDetection(void)
{
  // Fetching the service topic URI parameter
  if(!nh_.getParam("/rapp_human_detection_detect_humans_topic", humanDetectionTopic_))
  {
    ROS_ERROR("Human detection topic param does not exist");
  }
  // Creating the service server concerning the human detection functionality
  humanDetectionService_ = nh_.advertiseService(humanDetectionTopic_,
    &HumanDetection::humanDetectionCallback, this);
}

/**
 * @brief Serves the human detection ROS service callback
 * @param req [rapp_platform_ros_communications::HumanDetectionRosSrv::Request&] The ROS service request
 * @param res [rapp_platform_ros_communications::HumanDetectionRosSrv::Response&] The ROS service response
 * @return bool - The success status of the call
 */
bool HumanDetection::humanDetectionCallback(
  rapp_platform_ros_communications::HumanDetectionRosSrv::Request& req,
  rapp_platform_ros_communications::HumanDetectionRosSrv::Response& res)
{
  std::vector<unsigned int [4]> history;
  std::vector<cv::Rect> humans = human_detector_.findHuman2D(req.imageFilename); // run detectHuman2D
  for(unsigned int i = 0 ; i < humans.size() ; i++)
  {

    unsigned int temp[4];
    temp[0] = humans[i].x;
    temp[1] = humans[i].y;
    temp[2] = humans[i].x + humans[i].width;
    temp[3] = humans[i].y + humans[i].height;

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

	//responses
    res.humans_up_left.push_back(up_left_corner);
    res.humans_down_right.push_back(down_right_corner);
  }

  return true;
}
