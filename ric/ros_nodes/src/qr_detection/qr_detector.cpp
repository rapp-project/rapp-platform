#include <qr_detection/qr_detector.h>


QrDetector::QrDetector(void)
{
}

void QrDetector::findQrs(
  std::string file_name,
  std::vector<cv::Rect> &qr_points,
  std::vector<std::string> &qr_messages)
{
  cv::Mat input_img, grayscale_img;
  // Must check if file exists
  input_img = cv::imread(file_name);
  cv::cvtColor(input_img, grayscale_img, CV_BGR2GRAY);
}
