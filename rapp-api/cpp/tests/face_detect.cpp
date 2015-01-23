#include "../src/cloud/service_controller/service_controller.hpp"
#include "../src/cloud/faceDetector/faceDetector.hpp"

#include <opencv2/opencv.hpp>
#include <b64/encode.h>

#include <iostream>
#include <fstream>
#include <iomanip>

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
    /*
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
    */
    
    std::cout << "Opening image" << std::endl;
    std::ifstream image;
    image.open ( "picture.jpg", std::ios::in | std::ios::binary | std::ios::ate );
    
    if ( image.is_open() )
    {
        auto size = image.tellg();
        char * buffer = new char [size];
        image.seekg ( 0, std::ios::beg );
        image.read ( buffer, size );
        image.close();
        std::cout << "Read Image size: " << size << " bytes" << std::endl ;

        if ( image )
        {
            // Copy Bytes into a safer container
            //std::vector<char> data( buffer, buffer + size );
            
            // Ofstream will hold base-encoded picture
            std::ofstream basefile;
            
            base64::encoder E;
            E.encode( image, basefile );
            
            outfile.close();
        }
        else
            std::cout << "error: only " << image.gcount() << " could be read" << std::endl;

        delete[] buffer;
    }
    else
        std::cerr << "Error loading image" << std::endl;

    return 0;
}