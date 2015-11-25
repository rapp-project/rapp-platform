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

#ifndef RAPP_FACE_DETECTOR_NODE
#define RAPP_FACE_DETECTOR_NODE

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @class FaceDetector
 * @brief Class that implements a face detection algorithm based on
 * a Haar cascade classifier
 */
class FaceDetector
{
  public:

    /** 
     * @brief Default constructor
     */
    FaceDetector(void);

    /**
     * @brief   Loads an image from a file URL
     * @param   file_name [std::string] The image's file URL
     * @return  [cv::Mat] The image in OpenCV representation
     */
    cv::Mat loadImage(std::string file_name);

    /**
     * @brief   Finds faces in an image retrieved from a file URL
     * @param   file_name [std::string] The image file's URL
     * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
     *          Each face is represented by a rectangle.
     */
    std::vector<cv::Rect> findFaces(std::string file_name);

    /**
     * @brief   Detects faces from a cv::Mat
     * @param   input_img [const cv::Mat&] The input image
     * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
     *          Each face is represented by a rectangle.
     */
    std::vector<cv::Rect> detectFaces(const cv::Mat& input_img);

  private:

    /**
     * @brief   Detects faces from a cv::Mat
     * @param   input_img [const cv::Mat&] The input image
     * @param   haar_path [const std::string] The Haar training model path
     * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
     *          Each face is represented by a rectangle.
     */
    std::vector<cv::Rect> detectFaces(const cv::Mat& input_img,
      const std::string& haar_path);

    /**
     * @brief   Identify unique faces from two sets of faces
     * @param   frontFaceVector [const std::vector<cv::Rect>&] The first set of faces
     * @param   secondFaceVector [const std::vector<cv::Rect>&] The second set of faces
     * @return  [std::vector<cv::Rect>] A vector containing the unique faces.
     *          Each face is represented by a rectangle.
     */
    std::vector<cv::Rect> identifyUniqueFaces(
      const std::vector<cv::Rect> firstFaceVector,
      const std::vector<cv::Rect> secondFaceVector);
};

#endif
