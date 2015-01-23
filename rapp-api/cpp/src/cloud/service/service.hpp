#ifndef RAPP_SERVICE_CLOUD_SERVICE_ABC
#define RAPP_SERVICE_CLOUD_SERVICE_ABC
#include "Includes.ihh"

namespace rapp {
namespace services {
namespace cloud {

/**
 * @class service
 * @brief Abstract Base Class `service` is used internally for mechanism abstraction
 * @version 2
 * @date 18-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 */

class service
{
  public:
      
      /// @return an ASIO Service ABC socket, which will be scheduled as a single Job, or Part of a Job Group
      virtual std::shared_ptr<rapp::services::asio_socket> Job ( ) const = 0;
      
      /// Handle the obtained raw data from the TCP buffer
      virtual void handle ( boost::asio::streambuf & ) = 0;
      
      /// Destructor
      virtual ~service ( ) { }
      
      // Other future service abstraction interfacing should be placed here
};

}
}
}
#endif
