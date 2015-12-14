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

  Authors: Aris Thallas, Manos Tsardoulias
  contact: aris.thallas@{iti.gr, gmail.com}, etsardou@iti.gr

******************************************************************************/

#include <face_detection/face_detector.h>

/** 
 * @brief Default constructor
 */
FaceDetector::FaceDetector(void)
{
}

/**
 * @brief   Loads an image from a file URL
 * @param   file_name [std::string] The image's file URL
 * @return  [cv::Mat] The image in OpenCV representation
 */
cv::Mat FaceDetector::loadImage(std::string file_name)
{
  cv::Mat input_img;
  // Must check if file exists
  input_img = cv::imread(file_name);
  return input_img;
}

/**
 * @brief   Detects faces from a cv::Mat
 * @param   input_img [const cv::Mat&] The input image
 * @param   fast [bool] True for fast detection -- frontal only
 * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
 *          Each face is represented by a rectangle.
 */
std::vector<cv::Rect> FaceDetector::detectFaces(
  const cv::Mat& input_img, bool fast)
{
  std::vector<cv::Rect> front_faces, profile_faces, final_faces;
  cv::Mat grayscale_img;
  if( input_img.empty() )
  {
    return final_faces;
  }
  cv::cvtColor(input_img, grayscale_img, CV_BGR2GRAY);
  cv::equalizeHist(grayscale_img, grayscale_img);

  // Detect Front Faces
  std::string haar_file_path =
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
  front_faces = detectFaces( grayscale_img, haar_file_path );

  if(fast)
  {
    return front_faces;
  }

  // Detect Profile Faces
  haar_file_path = "/usr/share/opencv/haarcascades/haarcascade_profileface.xml";
  profile_faces = detectFaces( grayscale_img, haar_file_path );

  // Identify unique faces
  final_faces = identifyUniqueFaces( front_faces, profile_faces );

  return final_faces;
}

/**
 * @brief   Detects faces from a cv::Mat
 * @param   input_img [const cv::Mat&] The input image
 * @param   haar_path [const std::string] The Haar training model path
 * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
 *          Each face is represented by a rectangle.
 */
std::vector<cv::Rect> FaceDetector::detectFaces(const cv::Mat& input_img,
  const std::string& haar_path)
{
  std::vector<cv::Rect> faces, final_faces;

  // The Haar cascade classifier
  cv::CascadeClassifier face_cascade;
  // Create the classifier
  face_cascade.load(haar_path);

  face_cascade.detectMultiScale(input_img, faces, 1.1, 4);

  // If no faces were found make the algorithm less strict
  if(faces.size() == 0)
  {
    face_cascade.detectMultiScale(input_img, faces, 1.1, 3);
  }

  // Check the faces again to eliminate false positives
  for(unsigned int i = 0 ; i < faces.size() ; i++)
  {
    cv::Rect tmp_rect = faces[i];
    tmp_rect.x -= 10;
    tmp_rect.y -= 10;
    tmp_rect.width += 20;
    tmp_rect.height += 20;
    if(tmp_rect.x < 0 || tmp_rect.y < 0 ||
      (tmp_rect.x + tmp_rect.width) > input_img.size().width ||
      (tmp_rect.y + tmp_rect.height) > input_img.size().height)
    {
      continue;
    }
    cv::Mat temp_map = input_img(faces[i]);
    std::vector<cv::Rect> tmp_faces;
    face_cascade.detectMultiScale(temp_map, tmp_faces, 1.1, 3);
    if(tmp_faces.size() == 0)
    {
      continue;
    }
    final_faces.push_back(faces[i]);
  }
  return final_faces;
}

/**
 * @brief   Identify unique faces from two sets of faces
 * @param   frontFaceVector [const std::vector<cv::Rect>&] The first set of faces
 * @param   secondFaceVector [const std::vector<cv::Rect>&] The second set of faces
 * @return  [std::vector<cv::Rect>] A vector containing the unique faces.
 *          Each face is represented by a rectangle.
 */
std::vector<cv::Rect> FaceDetector::identifyUniqueFaces(
      const std::vector<cv::Rect> firstFaceVector,
      const std::vector<cv::Rect> secondFaceVector)
{
  std::vector<cv::Rect> final_faces;

  final_faces = firstFaceVector;

  final_faces.insert( final_faces.end(), secondFaceVector.begin(),
    secondFaceVector.end() );

  int size = final_faces.size();
  for( unsigned int i = 0; i < size; i++ )
  {
    final_faces.push_back( final_faces[i] );
  }
  groupRectangles( final_faces, 1, 0.2 );

  return final_faces;
}

/**
 * @brief   Finds faces in an image retrieved from a file URL
 * @param   file_name [std::string] The image file's URL
 * @param   fast [bool] True for fast detection -- frontal only
 * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
 *          Each face is represented by a rectangle.
 */
std::vector<cv::Rect> FaceDetector::findFaces(std::string file_name, bool fast)
{
  cv::Mat input_img;
  input_img = loadImage(file_name);
  return detectFaces(input_img, fast);
}
