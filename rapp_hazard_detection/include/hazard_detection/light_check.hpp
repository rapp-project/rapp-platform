/******************************************************************************
Copyright 2016 RAPP

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

  Authors: Maciej Stefańczyk
  contact: m.stefanczyk@elka.pw.edu.pl
  
******************************************************************************/

#ifndef RAPP_LIGHT_CHECK
#define RAPP_LIGHT_CHECK

#include <vector>
#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>

/**
 * Class implementing methods related with light checking behaviour. 
 */
class LightCheck {
public:
  /**
   * Check, whether the light is turned on. Light source should be placed
   * in the center of the image, and exposure must be set to rather low value.
   * 
   * \param fname path to the image file
   * \param debug if set, debug information is produced
   * 
   * \return estimated light level [0..100]
   */
  int process( const std::string & fname, bool debug = false );

protected:
  /**
   * Generate rectangle coordinates based on its center and size 
   * 
   * \param x rectangle center (x)
   * \param y rectangle center (y)
   * \param w rectangle width
   * \param h rectangle height
   * 
   * \return generated rectangle
   */
  cv::Rect centered_rect(int x, int y, int w, int h);

  /**
   * Compute average light level on given (sub)image.
   * 
   * \param img input image
   * 
   * \return average light level on the image
   */
  float average_light(cv::Mat img);
};

#endif /* RAPP_LIGHT_CHECK */
