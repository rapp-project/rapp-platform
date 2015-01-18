#include "ontologySubclassOf.hpp"

namespace rapp {
namespace services {
namespace cloud {


ontologySubclassOf::ontologySubclassOf (
                                         const std::string query
                                         std::function< void( std::vector<std:string> ) > callback
                                       )
: callback__ ( callback )
{   
    // Craft the POSt field
    std::string post = "query="+query+"\r\n\r\n";
    
    // Craft the actual header
    std::string header = "POST " + std::string( rapp::services::cloud::face_detect_uri ) + " HTTP/1.1\r\n";
    header += "Host: " + std::string( rapp::services::cloud::server_address ) + "\r\n";
    header += "Content-Type: application/x-www-form-urlencoded\r\n";
    
    // Adjust the POST Content-length
    header += "Content-Length: " + boost::lexical_cast<std::string>( post.length() ) + "\r\n";
    
    // Close connection after replying
    header += "Connection: close\r\n\r\n";
    
    // Finally, construct the asio_service_client using forged HEADER and POST
    client__ = std::make_shared<rapp::services::asio_service_client>( header, 
                                                                      post, 
                                                                      std::bind ( &ontologySubclassOf::handle, 
                                                                                  this,
                                                                                  std::placeholders::_1 ) );
}

std::shared_ptr<rapp::services::asio_service_client> ontologySubclassOf::Job ( ) const
{
    return client__;
}

void ::handle ( boost::asio::streambuf & buffer )
{   
    // Convert the buffer into a string
    std::string reply ( ( std::istreambuf_iterator<char>( &buffer ) ), std::istreambuf_iterator<char>() );
    
    // We will store the detected classes here
    std::vector<std:string> classes;
    
    /*
     * TODO: Treat the string as a JSON Array, and iterate string objects
     *       What I should be getting is a JSON like this:
     * 
     *          [{ "subclassOf" : "Entity" },{ "subclassOf" : "Object" }, { "superclassOf" : "Person" }]
     * 
     *       Basically, an Array of JSON string-objects, each with one string member.
     * 
     *       The JSON shall be parsed, and I will reconstruct either an std::vector<std::string>
     *       Or, if you prefer encapsulation or typedef, an std::vector<rapp::object::sublassOf>
     *                                                      std::vector<rapp::object::superclassOf>
     * 
     */
    
    // call the user-defined callback
    callback__ ( classes );
}

}
}
}