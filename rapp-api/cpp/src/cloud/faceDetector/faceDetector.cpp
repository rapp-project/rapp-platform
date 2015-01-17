#include "faceDetector.hpp"

namespace rapp
{
namespace services
{
namespace cloud
{
    
faceDetector::faceDetector ( 
                              cv::Mat image,
                              std::function< void( std::vector<rapp::object::face> objects ) > callback
                           )
:  callback__ ( callback )
{          
    // Serialise and base encode the image
    std::string post = preprocess__ ( image );
    
    // Craft the actual header
    std::string header = "POST " + std::string( rapp::services::cloud::face_detect_uri ) + " HTTP/1.1\r\n";
    header += "Host: " + std::string( rapp::services::cloud::server_address ) + "\r\n";
    header += "Content-Type: application/x-www-form-urlencoded\r\n";
    
    // Adjust the Content-length, this is needed for a correct Header!
    header += "Content-Length: " + boost::lexical_cast<std::string>( post.length() ) + "\r\n";
    
    std::cout << post << std::endl;
    
    // Close connection after receiving and responding
    header += "Connection: close\r\n\r\n";
    
    /*
     * Construct the asio_service_client - use the forged HEADER and POST data
     * We ask the asio client, to use our own ::handle as the callback that will receive the raw data response
     * service::handle is part of the service ABC, which all services must satisfy
     */
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
    /*
     * NOTE: Up to this point, we're receiving a streambuf.
     *  
     * If we want to use JSON (de)serialisation, then we need to handle the data as stringdata (images could be base64 encoded).
     * If on the other hand, we want to handle it as raw data, then we can simply try to load as binary/raw data in memory.
     * Realistically, I think we need some hybrid combo. This is due to the fact that we have meta-data (e.g., face `John`) as well as Raw data.
     * We can compartementalise the data from the meta-data.
     */
    
    // obtain the response buffer with the string-encoded faces and their meta-data
    std::string data ( ( std::istreambuf_iterator<char>( &buffer ) ), std::istreambuf_iterator<char>() );
    
    std::cout << data << std::endl;
    
    // Discovered & parsed
    std::vector<rapp::object::face> faces;
    
    /* 
     * TODO: Process the data, instantiate objects, store them in vector faces
     *       
     *       This assumes, correct (de)serialisation from the received buffer, into `face` objects.
     *       I cannot do this ATM, as it requires agreement upon what I will receive from the Service.
     */
    
    // call the user defined callback
    callback__( faces );
}

std::string faceDetector::preprocess__ ( const cv::Mat & image )
{   
    // TODO: Find out exactly how I can convert the cv::Mat to a stream, for base64 encoding
    const auto * input = ( const unsigned char * )( image.data );
    
    /*
    for ( int j = 0; j < image.rows; j++ )
    {
        for ( int i = 0; i < image.cols; i++ )
        {
            unsigned char b = input [ image.step * j + i ] ;
            unsigned char g = input [ image.step * j + i + 1 ];
            unsigned char r = input [ image.step * j + i + 2 ];
        }
    }
    base64::encoder E;
    E.encode(std::cin, std::cout);
    */
    
    return "TODO";
}


}
}
}