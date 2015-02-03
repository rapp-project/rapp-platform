#include <face_detection/face_detector.h>


FaceDetector::FaceDetector(void)
{
  std::string haar_file_path = 
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
  // Create the classifier
  face_cascade.load(haar_file_path);
}

std::vector<cv::Rect> FaceDetector::findFaces(std::string file_name)
{
  cv::Mat input_img, grayscale_img;
  // Must check if file exists
  input_img = cv::imread(file_name);
  cv::cvtColor(input_img, grayscale_img, CV_BGR2GRAY);
  cv::equalizeHist(grayscale_img, grayscale_img);
  std::vector<cv::Rect> faces;

  face_cascade.detectMultiScale(grayscale_img, faces, 1.1, 3,
    CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_SCALE_IMAGE, cv::Size(30,30));

  return faces;
}
