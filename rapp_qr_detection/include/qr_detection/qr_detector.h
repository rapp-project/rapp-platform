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

#include <zbar.h>

struct QrCode
{
  cv::Point qrcode_center;
  std::string qrcode_desc;
};

class QrDetector
{
  public:

    // Default constructor
    QrDetector(void);

    cv::Mat loadImage(std::string file_name);

    void detectQrs(
      const cv::Mat& img,
      std::vector<cv::Point> &qr_points,
      std::vector<std::string> &qr_messages
      );

    void findQrs(
      std::string file_name,
      std::vector<cv::Point> &qr_points,
      std::vector<std::string> &qr_messages
      );

    ~QrDetector(){}
};

#endif
