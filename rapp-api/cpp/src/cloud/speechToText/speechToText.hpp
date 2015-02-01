#ifndef RAPP_SERVICE_CLOUD_SPEECH_TO_TEXT
#define RAPP_SERVICE_CLOUD_SPEECH_TO_TEXT
#include "Includes.ihh"

namespace rapp {
namespace services {
namespace cloud {

/**
 * @class speechToText
 * @brief Asynchronous Service which will request the cloud to process speech-to-text
 * @version 0
 * @date 1-February-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * @see /cloud/globals/globals.hpp
 *      for parameters used, such as Server IP, Server Port, HOP URI
 */

class speechToText : public service
{
  public:

      /**
       * @brief Contrusct a speechToText handler
       * @param ? NOTE - What will we pass audio files as? File objects, raw bytes?
       * @param callback will be executed once the rapp cloud has responded
       */
      speechToText (
                     ???,
                     std::function< void( std::vector<std::string> words ) > callback
                   );

      /**
       * Get the async_client as a Job for the service scheduler
       * @warning You have to schedule the execution, it is not done automatically!
       */
      std::shared_ptr<rapp::services::asio_socket> Job ( ) const;

  private:

      /// Parse @param buffer received from the socket, into a vector of faces
      void handle ( boost::asio::streambuf & buffer );



      /// The callback called upon completion of receiving the detected words
      std::function< void( std::vector<std::string> words ) > callback__;
      
      /// The ASIO Client Socket used internally
      std::shared_ptr<rapp::services::asio_service_raw> client__;
};

}
}
}
#endif