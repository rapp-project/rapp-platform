#include <face_detection/face_detection.h>

FaceDetection::FaceDetection(void)
{
  faceDetectionTopic_ = "ric/face_detection_service";

  // Creating the service server concerning the face detection functionality
  faceDetectionService_ = nh_.advertiseService(faceDetectionTopic_, 
    &FaceDetection::faceDetectionCallback, this);

  // Create the classifier
  face_cascade.load("haarcascade_frontalface_alt.xml");
}

bool FaceDetection::faceDetectionCallback(
  rapp_platform_ros_communications::FaceDetectionRosSrv::Request& req,
  rapp_platform_ros_communications::FaceDetectionRosSrv::Response& res)
{
  cv::Mat input_img, grayscale_img;
  input_img = cv::imread(req.imageFilename.c_str());
  cv::cvtColor(input_img, grayscale_img, CV_BGR2GRAY);
  cv::equalizeHist(grayscale_img, grayscale_img);

  std::vector<cv::Rect> faces;

  face_cascade.detectMultiScale(grayscale_img, faces, 1.1, 3,
    CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, cv::Size(30,30));

  for(unsigned int i = 0 ; i < faces.size() ; i++)
  {
    geometry_msgs::PointStamped up_left_corner;
    geometry_msgs::PointStamped down_right_corner;

    up_left_corner.point.x = faces[i].x;
    up_left_corner.point.y = faces[i].y;

    down_right_corner.point.x = faces[i].x + faces[i].width;
    down_right_corner.point.y = faces[i].y + faces[i].height;
    
    res.faces_up_left.push_back(up_left_corner);
    res.faces_down_right.push_back(down_right_corner);
  }

  return true;
}

