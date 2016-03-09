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


HazardDetection::HazardDetection(void)
{
  // Fetching the service topic URI parameter
  if(!nh_.getParam("/rapp_hazard_detection_light_check_topic", lightCheckTopic_))
  {
    ROS_ERROR("Light check topic param does not exist");
  }
  
  // Creating the service server concerning the light_check functionality
  lightCheckService_ = nh_.advertiseService(lightCheckTopic_,
    &HazardDetection::lightCheckCallback, this);

  // Fetching the service topic URI parameter
  if(!nh_.getParam("/rapp_hazard_detection_door_check_topic", doorCheckTopic_))
  {
    ROS_ERROR("Door check topic param does not exist");
  }
  
  // Creating the service server concerning the door_check functionality
  doorCheckService_ = nh_.advertiseService(doorCheckTopic_,
    &HazardDetection::doorCheckCallback, this);
}

bool HazardDetection::lightCheckCallback(
      rapp_platform_ros_communications::LightCheckRosSrv::Request& req,
      rapp_platform_ros_communications::LightCheckRosSrv::Response& res )
{
  int light_level = light_check.process(req.imageFilename);
  res.light_level = light_level;
  return true;
}

bool HazardDetection::doorCheckCallback(
      rapp_platform_ros_communications::DoorCheckRosSrv::Request& req,
      rapp_platform_ros_communications::DoorCheckRosSrv::Response& res )
{
  int door_angle = door_check.process(req.imageFilename);
  res.door_angle = door_angle;
  return true;
}
