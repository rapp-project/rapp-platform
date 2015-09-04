#ifndef RAPP_QR_DETECTOR_NODE
#define RAPP_QR_DETECTOR_NODE

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <zbar.h>

struct QrCode
{
  cv::Point qrcode_center;
  std::string qrcode_desc;
};

class QrDetector
{
  public:

    // Default constructor
    QrDetector(void);

    cv::Mat loadImage(std::string file_name);

    void detectQrs(
      const cv::Mat& img,
      std::vector<cv::Point> &qr_points,
      std::vector<std::string> &qr_messages
      );

    void findQrs(
      std::string file_name,
      std::vector<cv::Point> &qr_points,
      std::vector<std::string> &qr_messages
      );

    ~QrDetector(){}

  private:

    //!< QrCode scanner
    zbar::ImageScanner scanner;
};

#endif
