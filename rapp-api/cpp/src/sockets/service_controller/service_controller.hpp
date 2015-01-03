#ifndef RAPP_SERVICE_CONTROLLER
#define RAPP_SERVICE_CONTROLLER
#include "Includes.ihh"

namespace rapp
{
namespace services
{

/**
 * @class service_controller
 * @brief Main class that controllers RAPP Services
 * @version 1
 * @date 21-December-2014
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * 
 */

class service_controller
{
  public:

    /// Default Constructor
    service_controller ( );
    
    /// Deleted constructor
    service_controller ( const service_controller & ) = delete;
    
    /// Deleted assignment operator
    service_controller& operator=( const service_controller & ) = delete;
    
    /// Get the Scheduler
    boost::asio::io_service & Scheduler ( );
    
    /// Get the Resolver
    boost::asio::ip::tcp::resolver::query & Resolver ( );
    

  private:
    
    /// Cloud Server Address
    const std::string __server;
    
    /// Boost Service IO Scheduler
    boost::asio::io_service __io_service;
      
    /// Endpoint Resolver
    boost::asio::ip::tcp::resolver::query __query;
};
}
}
#endif