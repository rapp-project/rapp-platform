#include "faceDetector.hpp"

namespace rapp {
namespace services {
namespace cloud {
    
faceDetector::faceDetector ( 
                              std::ifstream & image,
                              std::function< void( std::vector<std::pair<float,float>> ) > callback
                           )
:  callback__ ( callback )
{              
    // Send raw stream over the socket
    client__ = std::make_shared<rapp::services::asio_service_raw>( image,
                                                                   std::bind ( &faceDetector::handle, 
                                                                               this,
                                                                               std::placeholders::_1 ) );
}

faceDetector::faceDetector ( 
                              std::vector<char> & image,
                              std::function< void( std::vector<std::pair<float,float>> ) > callback
                           )
:  callback__ ( callback )
{              
    /*
    client__ = std::make_shared<rapp::services::asio_service_raw>( image, 
                                                                   std::bind ( &faceDetector::handle, 
                                                                               this,
                                                                               std::placeholders::_1 ) );
    */
}

std::shared_ptr<rapp::services::asio_socket> faceDetector::Job ( ) const
{
    return client__;
}

void faceDetector::handle ( boost::asio::streambuf & buffer )
{   
    // Convert the buffer into a string
    std::string reply ( ( std::istreambuf_iterator<char>( &buffer ) ), std::istreambuf_iterator<char>() );
    std::cout << reply << std::endl;

    // Discovered & parsed
    //std::vector<rapp::object::face> faces;
    std::vector<std::pair<float,float>> faces;
    
    /* 
     * TODO:  Treat the string as a JSON Array, and iterate coordinate objects
     *        What I should be getting, is a JSON, like this:
     * 
     *                  [{ "from_x" : 0.36, "from_y" : 3.45, "to_x": 5.45 "to_y" : 7.45  },{ ... }]
     * 
     *        Basically, an Array of JSON Objects, each with 4 members: from_x, from_y, to_x, to_y
     * 
     *        The JSON, shall be parsed, and I will reconstruct either the std::vector<std::pair<float,float>>
     *        Or if you prefer encapsulation or typedef, std::vector<rapp::object::face>
     */
    
    // call the user defined callback
    callback__( faces );
}



}
}
}
