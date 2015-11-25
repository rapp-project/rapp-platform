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

#ifndef RAPP_QR_DETECTOR_NODE
#define RAPP_QR_DETECTOR_NODE

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// QR detection utilized the zbar library
#include <zbar.h>

/**
 * @class QrCode
 * @brief Structure holding the essential info for a QR code
 */
struct QrCode
{
  cv::Point center;
  std::string message;
};

/**
 * @class QrDetector
 * @brief Provides the QR detection functionality
 */
class QrDetector
{
  public:

    /** 
     * @brief Default constructor 
     */
    QrDetector(void);

    /**
     * @brief Detects QRs in an image file
     * @param file_name [std::string] The input image URI
     * @return std::vector<QrCode> The detected QR codes
     */
    std::vector<QrCode> findQrs(std::string file_name);

    /**
     * @brief Detects QRs in a cv::Mat
     * @param img [const cv::Mat&] The input image in cv::Mat form
     * @return std::vector<QrCode> The detected QR codes
     */
    std::vector<QrCode> detectQrs(const cv::Mat& img);

  private:
    /**
     * @brief Loads an image into a cv::Mat structure
     * @param file_name [std::string] The file URI
     * @return cv::Mat
     */
    cv::Mat loadImage(std::string file_name);
};

#endif // RAPP_QR_DETECTOR_NODE
