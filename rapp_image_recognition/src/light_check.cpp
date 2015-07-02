#include <vector>
#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>

namespace LightCheck {

cv::Rect centered_rect(int x, int y, int w, int h) {
  return cv::Rect(x-w/2, y-h/2, w, h);
}

float average_light(cv::Mat img) {
  return cv::mean(img)[0];
}

int lightCheck( const std::string & fname, bool debug ) {
  cv::Mat img = cv::imread(fname);
  std::vector<cv::Rect> rois;
  std::vector<float> values;
  
  cv::Mat out = img.clone();
  
  rois.push_back(centered_rect(110, 80, 50, 50));
  rois.push_back(centered_rect(110, 240, 80, 80));
  rois.push_back(centered_rect(110, 400, 50, 50));
  rois.push_back(centered_rect(320, 80, 80, 80));
  rois.push_back(centered_rect(320, 400, 80, 80));
  rois.push_back(centered_rect(530, 80, 50, 50));
  rois.push_back(centered_rect(530, 240, 80, 80));
  rois.push_back(centered_rect(530, 400, 50, 50));
  rois.push_back(centered_rect(320, 240, 130, 130));
  
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
  }
  
  return res;
}

} 
