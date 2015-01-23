#ifndef RAPP_SERVICE_CLOUD_FACEDETECTOR
#define RAPP_SERVICE_CLOUD_FACEDETECTOR
#include "Includes.ihh"

namespace rapp {
namespace services {
namespace cloud {

/**
 * @class faceDetector
 * @brief Asynchronous Service which will request the cloud to detect faces
 * @version 3
 * @date 18-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * @warning We cannot assume the raw bytes are only the picture bytes.
 *          For example, a JPEG, has a header [0xFF][0xD8] and EOF [0xFF][0xD9]
 *          Each format uses different compression and headers.
 *          As such, this class is agnostic to exact Image Type, it will only transfer raw bytes.
 * 
 * @see /cloud/globals/globals.hpp
 *      for parameters used, such as Server IP, Server Port, HOP URI
 */
class faceDetector : public service
{
  public:
      
      /**
       * Constructor #1
       * 
       * @param image is an input stream representing the raw bytes of a picture.
       * @param callback is the functor that will receive a vector of detected face(s) coordinates
       * 
       * @note This is any asynchronous function.
       *       faceDetector::handler will execute on completion, and delegate results to callback functor.
       *       However, there is no promise on how long this will take, as processing times may vary!
       */
      faceDetector ( 
                     std::ifstream & image,
                     std::function< void( std::vector<std::pair<float,float>> ) > callback
                   );
      
      /**
       * Constructor #2
       * 
       * @param image is an input stream representing the raw bytes of a picture.
       * @param callback is the function that will receive a vector of detected face(s) coordinates
       * 
       * @note Since this is any asynchronous function, the handler method will be called upon completion.
       *       However, there is no promise on how long this will take, as processing times may vary.
       */
      faceDetector (
                     std::vector<char> & image,
                     std::function< void( std::vector<std::pair<float,float>> ) > callback
                   );
      
      /*
       * NOTE: I can encapsulate a Picture in a custom class: rapp::object::picture TODO???
      faceDetector (
                     std:shared_ptr<rapp::object::picture> image,
                     std::function< void( std::vector<rapp::object::face> objects ) > callback
                   );
      */
      
      /**
       * Get the async_client as a Job for the service scheduler
       * @warning You have to schedule the execution, it is not done automatically!
       */
      std::shared_ptr<rapp::services::asio_socket> Job ( ) const;
      
  private:
            
      /// Parse @param buffer received from the socket, into a vector of faces
      void handle ( boost::asio::streambuf & buffer );
            
      
      
      /// The callback called upon completion of receiving the detected faces
      //std::function< void( std::vector<rapp::object::face> objects ) > callback__;
      std::function< void( std::vector<std::pair<float,float>> faces) > callback__;
      
      /// The ASIO Client Socket used internally
      std::shared_ptr<rapp::services::asio_service_raw> client__;
};

}
}
}
#endif
