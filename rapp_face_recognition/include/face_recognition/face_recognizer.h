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

written by Jan Figat
******************************************************************************/

#ifndef RAPP_FACE_RECOGNIZER_NODE
#define RAPP_FACE_RECOGNIZER_NODE

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "vector"

#include <iostream>
#include <fstream>
#include <sstream>

#include <sys/types.h>
#include <sys/stat.h> //for checking if the directory exists
#include <unistd.h>
#include <pwd.h>

/**
 * @class FaceRecognizer
 * @brief Class that implements a face recognition algorithm based on
 * face recongition algorithms provided in OpenCV
 */
class FaceRecognizer
{
  public:

    /** 
     * @brief Default constructor
     */
    FaceRecognizer(void);

    /**
     * @brief   Recognizes a face while using detected faces and face model, which is loaded from a file URL
     * @param   img_ [cv::Mat&] The input image
     * @param   faces_ [std::vector< cv::Rect_<int> >] The vector containing detected faces in the given image
     * @param   model_name_ [const std::string]	The face model's URL
     * @param   predictedConfidenceVec [std::vector< double >&] Vector which will return predicted confidence value
     * @param   face_size_ [cv::Size] Size of face (default size is cv::Size(92,112))
     * @return  [std::vector< int >] The vector of recognized face ID
     */
    std::vector< int > recognizeFace(cv::Mat & img_, std::vector< cv::Rect_<int> > faces_, const std::string folder_, const std::string model_name_, std::vector< double > & predictedConfidenceVec, cv::Size face_size_ = cv::Size(92,112));
    
     /**
     * @brief   Detects humans from a cv::Mat
     * @param   filename [const std::string&] The input CSV file's URL
     * @param   images [std::vector<cv::Mat>&] The vector which will be filled with face's images
     * @param   labels [std::vector<int>&] The vector which will be filled with face's labels
     * @param   separator [char] The separator in CSV file (between image and label)
     */
    void read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, char separator = ';');


    /**
     * @brief   Learn face model
     * @param   fn_csv [std::string] Path to the CSV file with the face database
     * @return  [std::string] The model's URL
     */
    std::string learnFace(std::string fn_csv, const std::string folder_, const std::string model_name); //Path to the CSV file with the face database

    /**
     * @brief   Loads an image from a file URL
     * @param   file_name [std::string] The image's file URL
     * @return  [cv::Mat] The image in OpenCV representation
     */
    cv::Mat loadImage(std::string file_name);


  private:


};

#endif
