#include "../src/cloud/service_controller/service_controller.hpp"
#include "../src/cloud/faceDetector/faceDetector.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>


int main ( int argc, char* argv[] )
{
    rapp::services::service_controller ctrl;

    /**
     * @date 18-January-2015
     * @author Alex Gkiokas
     * 
     * One way of loading an image, is by getting it from the disk. We don't currently know how NAO acquires pictures, so this is tricky.
     * For that reason, class faceDetector has two constructors, one which takes as argument the input stream of the picture bytes,
     * and a second which takes as argument a vector<char>, where char is a byte.
     * We cannot use classes or structures such as cv::Mat, it makes no sense, as the cloud will need to deserialise the cv::Mat structure.
     * Instead, we can either send RAW BYTES, or BASE64 encoded Pictures.
     * Base64 is always larger than the actual picture, so it may NOT be ideal.
     * 
     * If NAO can obtain an image as an array of bytes (char * bytes = new char [size]) or as a vector of bytes (std::vector<char>),
     * then we can POST that data to the service.
     */
    
    std::cout << "Opening image" << std::endl;

    std::ifstream image;
    // NOTE: We need std::ios::ate to find the filestream buffer size in faceDetector. This is why we should encapsulate Images within a class
    image.open ( argv[1], std::ios::in | std::ios::binary | std::ios::ate );
    
    if ( image.is_open() )
    {
        // Create detect object
        auto fdetect = std::make_shared<rapp::services::cloud::faceDetector>( image, 
                                                                              [&]( std::vector<std::pair<float,float>> faces )
                                                                              {
                                                                                 std::cout << "found " << faces.size() << " faces!" << std::endl;
                                                                              });
        // Last, request from service controller to run this job
        ctrl.runJob ( fdetect->Job() );
    }
    else
        std::cerr << "Error loading image" << std::endl;

    return 0;
}
