#include "ros/ros.h"
#include "rapp_image_recognition/UploadImage.h"


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

std::string base_path;

bool service_UploadImage(rapp_image_recognition::UploadImage::Request  &req,
                         rapp_image_recognition::UploadImage::Response &res)
{
	cv_bridge::CvImagePtr cv_ptr;
	
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return false;
	}

	std::string fname = base_path;
	std::ostringstream ss;
	ss << ros::Time::now().toNSec();
	fname += ss.str();
	fname += ".png";

	ROS_INFO("Writing image %s", fname.c_str());
	cv::imwrite(fname, cv_ptr->image);
	
	res.path = fname;

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_objects_node");
	ros::NodeHandle n;

	if (!n.getParam("cloud_storage_path", base_path)) {
		base_path = "/tmp/";
	}

	ros::ServiceServer service = n.advertiseService("upload_image", service_UploadImage);
	ROS_INFO("Ready to receive images.");
	ros::spin();

	return 0;
}
