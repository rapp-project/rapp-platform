#ifndef RAPP_SERVICE_CLOUD_FACEDETECTOR
#define RAPP_SERVICE_CLOUD_FACEDETECTOR
#include "Includes.ihh"

namespace rapp {
namespace services {
namespace cloud {

/**
 * @class faceDetector
 * @brief Asynchronous Service which will request the cloud to detect faces
 * @version 4
 * @date 6-February-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * @see /cloud/globals/globals.hpp
 *      for parameters used, such as Server IP, Server Port, HOP URI
 */
class faceDetector : public service
{
  public:
      
      /**
       * @brief Constructor
       * @param image is an input stream representing the raw bytes of a picture.
       * @param callback is the function that will receive a vector of detected face(s) coordinates
       * 
       * @note Since this is any asynchronous function, the handler method will be called upon completion.
       *       However, there is no promise on how long this will take, as processing times may vary.
       */
      faceDetector (
                     std::shared_ptr<rapp::object::picture> image,
                     std::function< void( std::vector<std::pair<float,float>> ) > callback
                   );
      
      /**
       * Get the async_client as a Job for the service scheduler
       * @warning You have to schedule the execution, it is not done automatically!
       */
      std::shared_ptr<rapp::services::asio_socket> Job ( ) const;
      
  private:
            
      /// Parse @param buffer received from the socket, into a vector of faces
      void handle ( boost::asio::streambuf & buffer );
            
      
      
      /// The callback called upon completion of receiving the detected faces
      std::function< void( std::vector<std::pair<float,float>> ) > callback__;
      
      /// The ASIO Client Socket used internally
      std::shared_ptr<rapp::services::asio_service_raw> client__;
};

}
}
}
#endif
