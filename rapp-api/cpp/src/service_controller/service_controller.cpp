#include "service_controller.hpp"

namespace rapp
{
namespace services
{

service_controller::service_controller ( )
: __server( "localhost" ),
  __io_service( ),
  __query( __server, "http", boost::asio::ip::tcp::resolver::query::canonical_name )
{
}

boost::asio::io_service & service_controller::Scheduler ( )
{
    return __io_service;
}


boost::asio::ip::tcp::resolver::query & service_controller::Resolver ( )
{
    return __query;
}

}
}