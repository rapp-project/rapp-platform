#include "../src/cloud/service_controller/service_controller.hpp"
#include "../src/cloud/faceDetector/faceDetector.hpp"
#include "../src/objects/picture/picture.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>


int main ( int argc, char* argv[] )
{
    /**
     * @date 6-February-2015
     * @author Alex Gkiokas
     * 
     * This is now the default way of working with images
     */
    rapp::services::service_controller ctrl;
    
    std::cout << "Opening Picture" << std::endl;
    
    if ( auto pic = std::make_shared<rapp::object::picture>( "picture.jpg" ) )
    {
        std::cout << "Requesting face detection..." << std::endl;
        
        if ( auto fdetect = std::make_shared<rapp::services::cloud::faceDetector>( pic, 
            [&]( std::vector<std::pair<float,float>> faces )
            {
                std::cout << "found " << faces.size() << " faces!" << std::endl;
            }) )
        {
            // Last, request from service controller to run this job
            ctrl.runJob ( fdetect->Job() );
        }
    }
    else
        std::cerr << "Error loading image" << std::endl;

    return 0;
}