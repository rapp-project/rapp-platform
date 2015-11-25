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

#include <qr_detection/qr_detector.h>

/** 
 * @brief Default constructor 
 */
QrDetector::QrDetector(void)
{
}

/**
 * @brief Loads an image into a cv::Mat structure
 * @param file_name [std::string] The file URI
 * @return cv::Mat
 */
cv::Mat QrDetector::loadImage(std::string file_name)
{
  cv::Mat img = cv::imread(file_name);
  return img;
}

/**
 * @brief Detects QRs in a cv::Mat
 * @param img [const cv::Mat&] The input image in cv::Mat form
 * @return std::vector<QrCode> The detected QR codes
 */
std::vector<QrCode> QrDetector::detectQrs(const cv::Mat& input_frame)
{
  cv::Mat gray_frame;

  unsigned int channels = input_frame.channels();
  if( channels == 3 )
  {
    cv::cvtColor(input_frame, gray_frame, CV_BGR2GRAY);
  }

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

  //!< QrCode scanner
  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  scanner.scan(image);

  std::vector<QrCode> qrs;
  for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
    symbol != image.symbol_end(); ++symbol)
  {
    //QrCode detected_code;
    QrCode temp_qr;
    temp_qr.message = symbol->get_data();

    for(int i = 0; i < symbol->get_location_size(); i++)
    {
      temp_qr.center.x += symbol->get_location_x(i);
      temp_qr.center.y += symbol->get_location_y(i);
    }

    temp_qr.center.x /= symbol->get_location_size();
    temp_qr.center.y /= symbol->get_location_size();

    qrs.push_back(temp_qr);
  }
  return qrs;
}

/**
 * @brief Detects QRs in an image file
 * @param file_name [std::string] The input image URI
 * @return std::vector<QrCode> The detected QR codes
 */
std::vector<QrCode> QrDetector::findQrs(std::string file_name)
{
  cv::Mat input_frame;

  input_frame = loadImage(file_name);

  if( input_frame.empty() )
  {
    return std::vector<QrCode>();
  }

  return detectQrs(input_frame);
}
