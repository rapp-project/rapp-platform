#ifndef RAPP_SERVICE_CLOUD_FACEDETECTOR
#define RAPP_SERVICE_CLOUD_FACEDETECTOR
#include "Includes.ihh"

namespace rapp
{
namespace services
{
namespace cloud
{
    
/**
 * @class faceDetector
 * @brief Asynchronous Service which will request the cloud to detect faces
 * @version 2
 * @date 11-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 */
class faceDetector : public service
{
  public:
      
      /**
       * Detect face or faces
       * @param image is the OpenCV cv::Mat representing the 2D Matrix of pixel values (?)
       * @param callback is the function that will receive a vector of detected face(s)
       * 
       * @note Since this is any asynchronous function, the handler method will be called upon completion.
       *       However, there is no promise on how long this will take, as processing times may vary.
       */
      faceDetector ( 
                     cv::Mat image,
                     std::function< void( std::vector<rapp::object::face> objects ) > callback
                   );
      
      /**
       * Provide he async_client as a Job for the service scheduler
       * @warning You have to schedule the execution, it is not done automatically!
       * @return shared pointer
       */
      std::shared_ptr<rapp::services::asio_service_client> Job ( ) const;
      
  private:
            
      /// Parse @param buffer received from the socket, into a vector of faces
      void handle ( boost::asio::streambuf & buffer );
      
      /// Pre-Process @param image
      std::string preprocess__ ( const cv::Mat & image );
      
      ///
      std::string base64_encode ( unsigned char const * bytes_to_encode, unsigned int in_len );
      
      
      /// The callback called upon completion of receiving the detected faces
      std::function< void( std::vector<rapp::object::face> objects ) > callback__;
      
      /// The ASIO Client Socket used internally
      std::shared_ptr<rapp::services::asio_service_client> client__;
};

}
}
}
#endif