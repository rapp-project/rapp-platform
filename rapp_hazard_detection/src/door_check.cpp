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

  Authors: Maciej Stefańczyk
  contact: m.stefanczyk@elka.pw.edu.pl

******************************************************************************/

#include <hazard_detection/door_check.hpp>
#include <hazard_detection/Line.hpp>



int DoorCheck::process( const std::string & fname, DoorCheckParams params ) {
  cv::Mat img = cv::imread(fname, CV_LOAD_IMAGE_GRAYSCALE);
  
  // check, whether image is properly loaded
  if (img.empty()) return -1;

  // adaptive thresholding - results similar to edge detection
  cv::Mat img_thr;
  cv::adaptiveThreshold(img, img_thr, 255, params.thr_method, cv::THRESH_BINARY_INV, params.thr_block, params.thr_c);
  
  int prop_width = img.size().width;
  int prop_height = img.size().height;

  // detect line segments
  std::vector<cv::Vec4i> tmp_lines;
  cv::HoughLinesP( img_thr, tmp_lines, 1, CV_PI/180, params.hough_thr, params.hough_len, params.hough_gap);

  std::vector<Line> lines;
  for( size_t i = 0; i < tmp_lines.size(); i++ )
  {
    lines.push_back(Line(cv::Point(tmp_lines[i][0], tmp_lines[i][1]), cv::Point(tmp_lines[i][2], tmp_lines[i][3])));
  }
  
  // estimate door angle
  Line line;

  std::vector<Line> lines_v, lines_h;

  for (int i = 0; i < lines.size(); ++i) {
    line = lines[i];

    line.draw(img, cv::Scalar(255,255,255));
    if (abs(line.getAngle()) < M_PI/6) {
      lines_h.push_back(line);

      line.draw(img, cv::Scalar(128)); 
    } else
    if (abs(line.getAngle()) > 2*M_PI/6) {
      lines_v.push_back(line);
    } else {
      // skip line
    }
  }

  std::vector<cv::Point2f> points_v, points_h;

  /*for (int i = 0; i < 20; ++i) {
    std::random_shuffle(lines_v.begin(), lines_v.end());
  }*/

  // center line - vertical door frame
  Line * cl = NULL;

  // score of center line
  float cl_score = 0;

  for (int i = 0; i < lines_v.size(); ++i) {
    Line * tmp = &lines_v[i];

    // angle score - 1 for perfectly vertical line
    float angle_score = fabs(tmp->getAngle()) / (M_PI/2);
    
    // position score - 1 for centered line
    float mean_x = (tmp->getP1().x + tmp->getP2().x) / 2;
    float cx = 0.5 * prop_width;
    float position_score = 1.0 - fabs(cx - mean_x) / cx;
    
    // length score - 1 for line at least half of image height
    float length_score = 2 * tmp->length() / prop_height;
    if (length_score > 1) length_score = 1;
    
    float tmp_score = angle_score * position_score * length_score;
    if (tmp_score > cl_score) {
      cl = tmp;
      cl_score = tmp_score;
    }
  }

  if (cl)
    cl->draw(img, cv::Scalar(0, 0, 0));

  // left line - left floor/wall crossing
  Line * ll = NULL;

  // score of left line
  float ll_score = 0;

  for (int i = 0; i < lines_h.size(); ++i) {
    Line * tmp = &lines_h[i];
    
    // angle score - 1 for perfectly horizontal line
    float angle_score = (M_PI/2 - fabs(tmp->getAngle())) / (M_PI/2);
    
    // position score - 1 for line centered on the left part
    float mean_x = (tmp->getP1().x + tmp->getP2().x) / 2;
    float cx = 0.25 * prop_width;
    float position_score = 1.0 - fabs(cx - mean_x) / cx;
    // ignore lines laying on the right side 
    if (mean_x > 0.5 * prop_width) position_score = 0;
    
    // length score - 1 for line at least half of image width
    float length_score = 2 * tmp->length() / prop_width;
    if (length_score > 1) length_score = 1;
    
    float tmp_score = angle_score * position_score * length_score;
    if (tmp_score > ll_score) {
      ll = tmp;
      ll_score = tmp_score;
    }
  }

  if (ll)
    ll->draw(img, cv::Scalar(0));
    
  // right line - right floor/wall crossing
  Line * rl = NULL;

  // score of right line
  float rl_score = 0;

  for (int i = 0; i < lines_h.size(); ++i) {
    Line * tmp = &lines_h[i];
    
    // angle score - 1 for perfectly horizontal line
    float angle_score = (M_PI/2 - fabs(tmp->getAngle())) / (M_PI/2);
    
    // position score - 1 for line centered on the left part
    float mean_x = (tmp->getP1().x + tmp->getP2().x) / 2;
    float cx = 0.75 * prop_width;
    float position_score = 1.0 - fabs(cx - mean_x) / cx;
    // ignore lines laying on the left side 
    if (mean_x < 0.5 * prop_width) position_score = 0;
  
    // length score - 1 for line at least half of image width
    float length_score = 2 * tmp->length() / prop_width;
    if (length_score > 1) length_score = 1;
    
    float tmp_score = angle_score * position_score * length_score;
    if (tmp_score > rl_score) {
      rl = tmp;
      rl_score = tmp_score;
    }
  }

  if (rl)
    rl->draw(img, cv::Scalar(0));

  float angle = 0;
  if (ll && rl) {
    angle = fabs(ll->getAngle() - rl->getAngle()) * 180 / 3.1415;
  } else {
    angle = 90;
  }

  if (params.debug)
    cv::imwrite("/tmp/door_out.png", img);

  return angle;
}
