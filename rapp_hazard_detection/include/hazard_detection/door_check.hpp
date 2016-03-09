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

#ifndef RAPP_DOOR_CHECK
#define RAPP_DOOR_CHECK

#include <vector>
#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>

/**
 * Parameters for door checking behaviour.
 */
struct DoorCheckParams {
  // -------------------------------------------------------------------
  // Adaptive threshold parameters 
  // -------------------------------------------------------------------
  
  /// Adaptive thresholding algorithm to use, ADAPTIVE_THRESH_MEAN_C or ADAPTIVE_THRESH_GAUSSIAN_C.
  int thr_method;
  
  /// Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
  int thr_block;
  
  /// Constant subtracted from the mean or weighted mean. Normally, it is positive but may be zero or negative as well.
  int thr_c;
  
  // -------------------------------------------------------------------
  // Hough line detector parameters
  // -------------------------------------------------------------------
  
  /// Maximum allowed gap between points on the same line to link them.
  int hough_gap;
  
  /// Minimum line length. Line segments shorter than that are rejected.
  int hough_len;
  
  /// Accumulator threshold parameter. Only those lines are returned that get enough votes (>threshold).
  int hough_thr;
  
  // -------------------------------------------------------------------
  // Additional parameters
  // -------------------------------------------------------------------
  
  /// Debug flag. If set, additional output is produced.
  bool debug;
  
  // -------------------------------------------------------------------
  // Contructors
  // -------------------------------------------------------------------
  
  /// Default constructor with "optimal" parameters.
  DoorCheckParams() :
    thr_method(cv::ADAPTIVE_THRESH_GAUSSIAN_C),
    thr_block(3),
    thr_c(3),
    hough_gap(20),
    hough_len(60),
    hough_thr(80),
    debug(false)
  {}
};

/**
 * Class implementing methods related with door angle estimation behaviour. 
 */
class DoorCheck {
public:
  /**
   * Estimate angle of the door. Camera should be pointed to the contact
   * point of the door frame with the floor.
   * 
   * \param fname path to the image file
   * \param params processing parameters
   * 
   * \return estimateg door opening angle (in degrees) 
   */
  int process( const std::string & fname, DoorCheckParams params = DoorCheckParams() );

protected:
};

#endif /* RAPP_DOOR_CHECK */
