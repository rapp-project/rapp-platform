#ifndef RAPP_FACE_DETECTOR_NODE
#define RAPP_FACE_DETECTOR_NODE

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @class FaceDetector
 * @brief Class that implements a face detection algorithm based on
 * a Haar cascade classifier
 */
class FaceDetector
{
  public:

    // Default constructor
    FaceDetector(void);
    
    /**
     * @brief   Loads an image from a file URL
     * @param   file_name [std::string] The image's file URL
     * @return  [cv::Mat] The image in OpenCV representation
     */
    cv::Mat loadImage(std::string file_name);   

    /**
     * @brief   Finds faces in an image retrieved from a file URL
     * @param   file_name [std::string] The image file's URL
     * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
     *          Each face is represented by a rectangle.
     */
    std::vector<cv::Rect> findFaces(std::string file_name);

    /**
     * @brief   Detects faces from a cv::Mat
     * @param   input_img [const cv::Mat&] The input image
     * @return  [std::vector<cv::Rect>] A vector containing the detected faces.
     *          Each face is represented by a rectangle.
     */
    std::vector<cv::Rect> detectFaces(const cv::Mat& input_img);

  private:

};

#endif
