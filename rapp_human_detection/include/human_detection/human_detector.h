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

#ifndef RAPP_HUMAN_DETECTOR_NODE
#define RAPP_HUMAN_DETECTOR_NODE

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp> // for the svm algorithm
#include "opencv2/objdetect/objdetect.hpp" // for HOGDescriptor

/**
 * @class HumanDetector
 * @brief Class that implements a human detection algorithm based on
 * a Haar cascade classifier
 */
class HumanDetector
{
  public:

    /** 
     * @brief Default constructor
     */
    HumanDetector(void);

    /**
     * @brief   Loads an image from a file URL
     * @param   file_name [std::string] The image's file URL
     * @return  [cv::Mat] The image in OpenCV representation
     */
    cv::Mat loadImage(std::string file_name);

    /**
     * @brief   Finds humans in an image retrieved from a file URL
     * @param   file_name [std::string] The image file's URL
     * @return  [std::vector<cv::Rect>] A vector containing the detected humans.
     *          Each human is represented by a rectangle.
     */
    std::vector<cv::Rect> findHuman2D(std::string file_name);

    /**
     * @brief   Detects humans from a cv::Mat
     * @param   input_img [const cv::Mat&] The input image
     * @return  [std::vector<cv::Rect>] A vector containing the detected humans.
     *          Each human is represented by a rectangle.
     */
    std::vector<cv::Rect> detectHuman2D(const cv::Mat& input_img);

  private:

    /**
     * @brief   Detects humans from a cv::Mat
     * @param   input_img [const cv::Mat&] The input image
     * @param   haar_path [const std::string] The Haar training model path
     * @return  [std::vector<cv::Rect>] A vector containing the detected humans.
     *          Each human is represented by a rectangle.
     */
    std::vector<cv::Rect> detectHuman2D(const cv::Mat& input_img,
      const std::string& haar_path);

    /**
     * @brief   Identify unique humans from two sets of humans
     * @param   pedestrianVector [const std::vector<cv::Rect>&] The first set of humans
     * @param   upperbodyVector [const std::vector<cv::Rect>&] The second set of humans
     * @return  [std::vector<cv::Rect>] A vector containing the unique humans.
     *          Each human is represented by a rectangle.
     */
    std::vector<cv::Rect> identifyUniqueHumans(
      const std::vector<cv::Rect> pedestrianVector,
      const std::vector<cv::Rect> upperbodyVector);
};

#endif
