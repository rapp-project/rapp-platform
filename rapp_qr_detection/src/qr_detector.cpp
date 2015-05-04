#include <qr_detection/qr_detector.h>


QrDetector::QrDetector(void)
{
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

void QrDetector::findQrs(
  std::string file_name,
  std::vector<cv::Point> &qr_points,
  std::vector<std::string> &qr_messages)
{
  qr_points.clear();
  qr_messages.clear();

  cv::Mat input_frame, gray_frame;
  // Must check if file exists
  input_frame = cv::imread(file_name);
  cv::cvtColor(input_frame, gray_frame, CV_BGR2GRAY);

  int gaussiansharpenblur = 5;
  float gaussiansharpenweight = 0.8;

  cv::Mat blured;
  normalize(gray_frame, gray_frame, 255, 0, cv::NORM_MINMAX);
  cv::GaussianBlur(gray_frame, blured, cv::Size(0, 0), gaussiansharpenblur);
  cv::addWeighted(gray_frame, 1 + gaussiansharpenweight, blured,
    -gaussiansharpenweight, 0, gray_frame);

  int width = gray_frame.cols;
  int height = gray_frame.rows;
  uchar *raw = gray_frame.data;

  zbar::Image image(width, height, "Y800", raw, width * height);

  scanner.scan(image);
  qrcode_list.clear();

  for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
    symbol != image.symbol_end(); ++symbol)
  {
    //QrCode detected_code;
    qr_messages.push_back(symbol->get_data());

    cv::Point detected_center;
    for(int i = 0; i < symbol->get_location_size(); i++)
    {
      detected_center.x += symbol->get_location_x(i);
      detected_center.y += symbol->get_location_y(i);
    }

    detected_center.x /= symbol->get_location_size();
    detected_center.y /= symbol->get_location_size();

    qr_points.push_back(detected_center);
  }
}
