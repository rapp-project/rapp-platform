#include "faceDetector.hpp"

namespace rapp
{
namespace services
{
namespace cloud
{
    
faceDetector::faceDetector ( 
                              rapp::services::service_controller & controller,
                              const cv::Mat & image,
                              std::function< void( std::vector<rapp::object::face> objects ) > callback
                           )
:  __callback ( callback )
{ 
     
    // WARNING - This assumes we're using a PHP/apache2 server. Eventually will either use a HOP server. ADJUST ACCORDINGLY!
    const std::string page = "/facedetector.php";
    
    /// NOTE: Create the POST field: Username, Password, Image
    const std::string post =  "user=alex";
    post += "&pwd=qwepoi";
    
    post += "&image=" + __encode ( image ); // baseEncode64 the raw image?
    
    // Craft the actual header
    std::string header = "POST " + page + " HTTP/1.1\r\n";
    header += "Host: " + controller.Server() + "\r\n";
    header += "Content-Type: application/x-www-form-urlencoded\r\n";
    header += "Content-Length: " + boost::lexical_cast<std::string>( post.length() ) + "\r\n";
    header += "Connection: close\r\n\r\n";
    
    rapp::services::asio_service_client client ( controller.Resolver(), 
                                                 header, 
                                                 post,
                                                 std::bind ( &faceDetector::__parser, this, std::placeholders::_1 ) );
}


std::string faceDetector::__encode ( const cv::Mat & image )
{
    // TODO: base encode 64 the image regardless of endianess or CPU arch.
    // SEE: http://www.cryptopp.com/docs/ref/class_base64_encoder.html
    return "TODO";
}


void faceDetector::__parser ( boost::asio::streambuf & buffer )
{
    // obtain the response buffer with the string-encoded faces and their meta-data
    std::string data ( ( std::istreambuf_iterator<char>( &buffer ) ), std::istreambuf_iterator<char>() );
    
    // Discovered & parsed
    std::vector<rapp::object::face> faces;
    
    // TODO: Process the data, instantiate objects, store them in vector faces
    
    // call the user defined callback
    __callback( faces );
}


}
}
}