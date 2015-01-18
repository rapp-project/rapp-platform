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
    // TODO Serialise and base encode the image ? If not, then we have to POST Raw Bytes
    std::string post = base64__ ( image );
    
    // Craft the actual header
    std::string header = "POST " + std::string( rapp::services::cloud::face_detect_uri ) + " HTTP/1.1\r\n";
    header += "Host: " + std::string( rapp::services::cloud::server_address ) + "\r\n";
    header += "Content-Type: application/x-www-form-urlencoded\r\n";
    
    // TODO Adjust the Content-length, this is needed for a correct Header!
    header += "Content-Length: " + boost::lexical_cast<std::string>( post.length() ) + "\r\n";
    
    // Close connection after replying
    header += "Connection: close\r\n\r\n";
    
    // Finally, construct the asio_service_client using forged HEADER and POST
    client__ = std::make_shared<rapp::services::asio_service_client>( header, 
                                                                      post, 
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
    // TODO Serialise and base encode the image ? If not, then we have to POST Raw Bytes
    std::string post = base64__ ( image );
    
    // Craft the actual header
    std::string header = "POST " + std::string( rapp::services::cloud::face_detect_uri ) + " HTTP/1.1\r\n";
    header += "Host: " + std::string( rapp::services::cloud::server_address ) + "\r\n";
    header += "Content-Type: application/x-www-form-urlencoded\r\n";
    
    // TODO Adjust the Content-length, this is needed for a correct Header!
    header += "Content-Length: " + boost::lexical_cast<std::string>( post.length() ) + "\r\n";
    
    // Close connection after replying
    header += "Connection: close\r\n\r\n";
    
    // Finally, construct the asio_service_client using forged HEADER and POST
    client__ = std::make_shared<rapp::services::asio_service_client>( header, 
                                                                      post, 
                                                                      std::bind ( &faceDetector::handle, 
                                                                                  this,
                                                                                  std::placeholders::_1 ) );
}

std::shared_ptr<rapp::services::asio_service_client> faceDetector::Job ( ) const
{
    return client__;
}

void faceDetector::handle ( boost::asio::streambuf & buffer )
{   
    // Convert the buffer into a string
    std::string reply ( ( std::istreambuf_iterator<char>( &buffer ) ), std::istreambuf_iterator<char>() );
    
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

std::string faceDetector::base64__ ( std::ifstream & image )
{   
    // TODO... Base64 encode
    return "TODO";
}

std::string faceDetector::base64__ ( std::vector<char> & image )
{   
    // TODO... Base64 encode
    return "TODO";
}




}
}
}