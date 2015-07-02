#include "ros/ros.h"
#include "rapp_image_recognition/LightCheck.h"

#include "rapp_image_recognition/light_check.hpp"

bool service_LightCheck(rapp_image_recognition::LightCheck::Request  &req,
                        rapp_image_recognition::LightCheck::Response &res)
{
  res.result = LightCheck::lightCheck(req.fname);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_objects_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("light_check", service_LightCheck);
  ROS_INFO("Ready for light checking.");
  ros::spin();

  return 0;
}
