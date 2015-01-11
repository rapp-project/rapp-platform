#include "../src/cloud/service_controller/service_controller.hpp"
#include "../src/cloud/faceDetector/faceDetector.hpp"

#include <opencv2/opencv.hpp>

using namespace cv;

int main ( int argc, char* argv[] )
{
    // First, Instantiate a Service Controller
    rapp::services::service_controller ctrl;
    
    /* Second, Load an Image (either from file, or from Camera - up to you
     * 
     * NOTE: You may load an image from a RAW buffer in memory, using: cv::imdecode
     *       You may also, load an image from file, or somehow allocate from a live video feed (camera?)   
     */
    Mat image;
    image = imread( "picture.jpg", CV_LOAD_IMAGE_COLOR );   // Read the file


    if( !image.data )                                       // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    
    // Third, allocate a new faceDecector object
    auto fdetect = std::make_shared<rapp::services::cloud::faceDetector>( image, 
                                                                          [&]( std::vector<rapp::object::face> objects )
                                                                          {
                                                                            std::cout << "found " << objects.size() << " faces!" << std::endl;
                                                                          } );
    
    // Last, request from service controller to run this job
    ctrl.runJob ( fdetect->Job() );

    return 0;
}