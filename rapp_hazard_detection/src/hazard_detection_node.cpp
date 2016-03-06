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

#include <hazard_detection/hazard_detection.hpp>

/**
 * @brief The executable's main function.
 * Creates a ROS multispinner to enable concurrent requests and declares a HazardDetection object
 */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "face_detection_node");
  HazardDetection hdnode;

  ros::NodeHandle nh;
  int threads = 1;
  if(!nh.getParam("/rapp_hazard_detection_threads", threads))
  {
    ROS_ERROR("Hazard detection threads param not found");
  }
  else if(threads < 0)
  {
    threads = 1;
  }
  ros::MultiThreadedSpinner spinner(threads);
  spinner.spin();
  return 0;
}
