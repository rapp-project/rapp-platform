#ifndef RAPP_QR_DETECTOR_NODE
#define RAPP_QR_DETECTOR_NODE

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class QrDetector
{
  public:

    // Default constructor
    QrDetector(void);

    void findQrs(
      std::string file_name,
      std::vector<cv::Rect> &qr_points,
      std::vector<std::string> &qr_messages
      );

  private:

};

#endif
