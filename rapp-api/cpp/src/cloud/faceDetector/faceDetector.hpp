#ifndef RAPP_SERVICE_CLOUD_FACEDETECTION
#define RAPP_SERVICE_CLOUD_FACEDETECTION
#include "Includes.ihh"

namespace rapp
{
namespace services
{
namespace cloud
{
    
/**
 * @class faceDetector
 * @brief Asynchronous Service which will try to detect face(s) in an image
 * @version 1
 * @date 3-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * NOTE: This class can be re-used. We assume that user will handle the objects & memory of returned faces.
 *       This class, will keep spawning new sockets and connecting to the cloud service.
 *       We may also multi-thread its operations, in a FIFO-styled queue, or make it thread-safe and assume the user
 *       does the multi-threading via calling faceDetector::Find multiple times
 * 
 * NOTE: Serialising cv::Mat => https://cheind.wordpress.com/2011/12/06/serialization-of-cvmat-objects-using-boost/
 */
class faceDetector
{
  public:
      
      /**
       * Detect face or faces
       * @param controller is the master service controller object
       * @param image is the OpenCV cv::Mat representing the 2D Matrix of pixel values (?)
       * @param callback is the function that will receive a vector of detected face(s)
       * 
       * @note Since this is any asynchronous function, the handler method will be called upon completion.
       *       However, there is no promise on how long this will take, as processing times may vary.
       */
      faceDetector ( 
                     rapp::services::service_controller & controller,
                     const cv::Mat & image,
                     std::function< void( std::vector<rapp::object::face> objects ) > callback
                   );
      
  private:
      
      /// Encode into base64 regardless of endianess @param image
      std::string __encode ( const cv::Mat & image );
      
      /// Parse @param reply from face detector service, Produce a vector of face(s)
      void __parser ( boost::asio::streambuf & buffer );
      
      /// The callback called upon completion of receiving the detected faces
      std::function< void( std::vector<rapp::object::face> objects ) > __callback
      
      /// service controller used for accessing the cloud
      rapp::services::service_controller & __controller;
};

}
}
}
#endif