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

#include <hazard_detection/light_check.hpp>

int LightCheck::process( const std::string & fname, bool debug ) {
  cv::Mat img = cv::imread(fname);
  
  if (img.empty()) return -1;
  
  std::vector<cv::Rect> rois;
  std::vector<float> values;
  
  cv::Mat out = img.clone();
  
  int sx = img.size().width / 4;
  int sy = img.size().height / 4;
  int ss = img.size().height / 10;
  int sm = img.size().height / 6;
  int sl = img.size().height / 4;
  
  
  rois.push_back(centered_rect(sx,   sy, ss, ss));
  rois.push_back(centered_rect(sx, 2*sy, sm, sm));
  rois.push_back(centered_rect(sx, 3*sy, ss, ss));
  rois.push_back(centered_rect(2*sx,   sy, sm, sm));
  rois.push_back(centered_rect(2*sx, 3*sy, sm, sm));
  rois.push_back(centered_rect(3*sx,   sy, ss, ss));
  rois.push_back(centered_rect(3*sx, 2*sy, sm, sm));
  rois.push_back(centered_rect(3*sx, 3*sy, ss, ss));
  
  rois.push_back(centered_rect(2*sx, 2*sy, sl, sl));
  
  char buf[256];
  
  float vmax = 0, vmin = 255, vsum = 0;
  for (int i = 0; i < rois.size(); ++i) {
    float val = average_light(img(rois[i]));
    values.push_back(val);
    
    if (val < vmin) vmin = val;
    if (val > vmax) vmax = val;
    
    vsum += val;
        
    if (debug) {
      sprintf(buf, "%6.0f", values[i]);
      if (i < rois.size() - 1) {
        cv::rectangle(out, rois[i], cv::Scalar(0, 0, 256), 3);
      } else {
        cv::rectangle(out, rois[i], cv::Scalar(256, 0, 0), 3);
      }
      cv::putText(out, buf, cv::Point(rois[i].x + 5, rois[i].y + rois[i].height - 5), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
      std::cout << values[i] << std::endl;
    } // if (debug)
  }
  
  float center_val = values.back();
  vsum = vsum - vmin - vmax - center_val;
  float vavg = vsum / 6;
  float res = center_val - vavg;
  res = res * 4;
  if (res < 0) res = 0;
  if (res > 100) res = 100;
  
  if (debug) {
    std::cout << "C: " << center_val << "\n";
    std::cout << "m: " << vmin << "\n";
    std::cout << "M: " << vmax << "\n";
    std::cout << "s: " << vsum << "\n";
  }
  
  if (debug) {
    out.copyTo(img);
    cv::imshow("out", out);
    cv::waitKey(10000);
  }
  
  return res;
}


cv::Rect LightCheck::centered_rect(int x, int y, int w, int h) {
  return cv::Rect(x-w/2, y-h/2, w, h);
}

float LightCheck::average_light(cv::Mat img) {
  return cv::mean(img)[0];
}
