#include "service_controller.hpp"

namespace rapp
{
namespace services
{

std::mutex service_controller::service_mtx_;

    
service_controller::service_controller ( )
: server_ ( "localhost" ),      /// WARNING - This should be correctly pointing to http(s)://api.rapp.cloud
  io_service_ ( ),
  query_ ( server_, "9001" ), // "http"
  resolver_ ( io_service_ )
{
    //work_ = std::make_shared<boost::asio::io_service::work>( io_service_ );
}

boost::asio::io_service & service_controller::queue ( )
{
    return io_service_;
}

void service_controller::runJob ( const std::shared_ptr<asio_socket> job )
{
    // WARNING : if synchronicity gives us problems here, then allocate a new io_service, and use it within scope
    
    if ( !job )
        throw std::runtime_error ( "service_controller::runJob => param job is null" );
    
    job->Schedule( query_, resolver_, io_service_ );
    io_service_.run();
    std::lock_guard<std::mutex> lock ( service_mtx_ );
    io_service_.reset();
}

void service_controller::runJobs ( std::vector<std::shared_ptr<asio_socket>> jobs )
{
    // WARNING : if synchronicity gives us problems here, then allocate a new io_service, and use it within scope
    
    for ( const auto & job : jobs )
    {
        if ( !job )
            throw std::runtime_error ( "service_controller::runJobs => job in vector is null" );
        
        job->Schedule( query_, resolver_, io_service_ );
    }
    io_service_.run();
    std::lock_guard<std::mutex> lock ( service_mtx_ );
    io_service_.reset();
}


}
}
