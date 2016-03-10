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

#include <human_detection/human_detector.h>

/** 
 * @brief Default constructor
 */
HumanDetector::HumanDetector(void)
{
}

/**
 * @brief   Loads an image from a file URL
 * @param   file_name [std::string] The image's file URL
 * @return  [cv::Mat] The image in OpenCV representation
 */
cv::Mat HumanDetector::loadImage(std::string file_name)
{
  cv::Mat input_img;
  // Must check if file exists
  input_img = cv::imread(file_name);
  return input_img;
}

/**
 * @brief   Detects humans from a cv::Mat
 * @param   input_img [const cv::Mat&] The input image
 * @return  [std::vector<cv::Rect>] A vector containing the detected humans.
 *          Each human is represented by a rectangle.
 */
std::vector<cv::Rect> HumanDetector::detectHuman2D(const cv::Mat& input_img)
{
  std::vector<cv::Rect> pedestrian, upperbody, final_humans;
  cv::Mat grayscale_img;
  if( input_img.empty() )
  {
    return final_humans;
  }
  cv::cvtColor(input_img, grayscale_img, CV_BGR2GRAY);
  cv::equalizeHist(grayscale_img, grayscale_img);

  // Detect Pedestrians
  //std::vector<cv::Rect> found, found_filtered;
  cv::HOGDescriptor hog;
  hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
  hog.detectMultiScale(grayscale_img, pedestrian, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);
  
  // Detect Human Upperbody
  std::string haar_file_path = "/usr/share/opencv/haarcascades/haarcascade_upperbody.xml";
  upperbody = detectHuman2D( grayscale_img, haar_file_path );
  
  // Identify unique humans
  final_humans = identifyUniqueHumans( pedestrian, upperbody );

  return final_humans;
}

/**
 * @brief   Detects humans from a cv::Mat
 * @param   input_img [const cv::Mat&] The input image
 * @param   haar_path [const std::string] The Haar training model path
 * @return  [std::vector<cv::Rect>] A vector containing the detected humans.
 *          Each human is represented by a rectangle.
 */
std::vector<cv::Rect> HumanDetector::detectHuman2D(const cv::Mat& input_img,
  const std::string& haar_path)
{
  std::vector<cv::Rect> found, final_humans;

  // The Haar cascade classifier
  cv::CascadeClassifier human_cascade; // body_cascade
  // Create the classifier
  human_cascade.load(haar_path);

  // Parameters of detectMultiscale Cascade Classifier
  int groundThreshold = 2;
  double scaleStep = 1.1;
  cv::Size minimalObjectSize = cv::Size(100, 100); // minimal size of the detected object
  cv::Size maximalObjectSize = cv::Size(800, 800); // maximal size of the detected object
  // Detect humans
  human_cascade.detectMultiScale(input_img, found, scaleStep, groundThreshold, 0 | cv::CASCADE_SCALE_IMAGE, minimalObjectSize);//, maximalObjectSize);

  //human_cascade.detectMultiScale(input_img, found, 1.1, 4);

  // If no humans were found make the algorithm less strict
  //if(found.size() == 0)
  //{
  // /human_cascade.detectMultiScale(input_img, found, 1.1, 3);
  //}

  // Check the humans again to eliminate false positives
  for(unsigned int i = 0 ; i < found.size() ; i++)
  {
    cv::Rect tmp_rect = found[i];
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
    cv::Mat temp_map = input_img(found[i]);
    std::vector<cv::Rect> tmp_found;
    human_cascade.detectMultiScale(temp_map, tmp_found, 1.1, 3);
    if(tmp_found.size() == 0)
    {
      continue;
    }
    final_humans.push_back(found[i]);
  }
  return final_humans;
}

/**
 * @brief   Identify unique humans from two sets of humans
 * @param   pedestrianVector [const std::vector<cv::Rect>&] The first set of humans
 * @param   upperbodyVector [const std::vector<cv::Rect>&] The second set of humans
 * @return  [std::vector<cv::Rect>] A vector containing the unique humans.
 *          Each human is represented by a rectangle.
 */
std::vector<cv::Rect> HumanDetector::identifyUniqueHumans(
      const std::vector<cv::Rect> pedestrianVector,
      const std::vector<cv::Rect> upperbodyVector)
{
  std::vector<cv::Rect> final_vector;

  final_vector = pedestrianVector;

  final_vector.insert( final_vector.end(), upperbodyVector.begin(),
    upperbodyVector.end() );

  int size = final_vector.size();
  for( unsigned int i = 0; i < size; i++ )
  {
    final_vector.push_back( final_vector[i] );
  }
  cv::groupRectangles( final_vector, 1, 0.2 ); //basic set

  return final_vector;
}

/**
 * @brief   Finds humans in an image retrieved from a file URL
 * @param   file_name [std::string] The image file's URL
 * @return  [std::vector<cv::Rect>] A vector containing the detected humans.
 *          Each human is represented by a rectangle.
 */
std::vector<cv::Rect> HumanDetector::findHuman2D(std::string file_name)
{
  cv::Mat input_img;
  input_img = loadImage(file_name);
  //std::vector<cv::Rect> final_humans;
  //final_humans = detectHuman2D(input_img);
  return detectHuman2D(input_img);
}

