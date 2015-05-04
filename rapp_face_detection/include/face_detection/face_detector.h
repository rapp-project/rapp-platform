#ifndef RAPP_FACE_DETECTOR_NODE
#define RAPP_FACE_DETECTOR_NODE

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class FaceDetector
{
  public:

    // Default constructor
    FaceDetector(void);
   
    std::vector<cv::Rect> findFaces(std::string file_name);

  private:

    cv::CascadeClassifier face_cascade;
};

#endif
