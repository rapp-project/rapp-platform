#include "asio_service_raw.hpp"

namespace rapp {
namespace services {


asio_service_raw::asio_service_raw ( const std::vector<rapp::types::byte> & bytearray )
{
    bytes_ = bytearray;
}
    
asio_service_raw::asio_service_raw (
                                      const std::vector<rapp::types::byte> & bytearray,
                                      std::function<void( boost::asio::streambuf & )> callback
                                   )
: callback_ ( callback )
{
    bytes_ = bytearray;
}

void asio_service_raw::Schedule (
                                  boost::asio::ip::tcp::resolver::query & query,
                                  boost::asio::ip::tcp::resolver & resolver,
                                  boost::asio::io_service & io_service
                                )
{

    socket_ = std::unique_ptr<boost::asio::ip::tcp::socket>( new boost::asio::ip::tcp::socket( io_service ) );
    std::ostream request_stream ( &request_ );
    
    // Starting Delimiter is DAT
    // request_stream << "<DAT>"; - DEPRECATED
    
    for ( const auto & byte : bytes_ )
        request_stream << byte;
    
    request_stream << "</EOF!>";
    // Ending Delimiter is /EOF!
        
    //std::string raw ( ( std::istreambuf_iterator<char>( &request_ ) ), std::istreambuf_iterator<char>() );
    //std::cout << raw << std::endl;

    resolver.async_resolve( query,
                            boost::bind( & asio_service_raw::handle_resolve,
                                          this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::iterator ) );
}

void asio_service_raw::handle_reply ( )
{   
    // NOTE: Why cast in string??? 
    std::string raw ( ( std::istreambuf_iterator<char>( &response_ ) ), std::istreambuf_iterator<char>() );
    std::cout << raw << std::endl; // REMOVE
}

void asio_service_raw::error_handler ( const boost::system::error_code & error )
{
    std::cerr << error.message() << std::endl;
}

void asio_service_raw::handle_resolve ( 
                                        const boost::system::error_code & err,
                                        boost::asio::ip::tcp::resolver::iterator endpoint_iterator
                                      )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_raw::handle_resolve socket ptr null" );
    
    if (!err)
    {
        // Attempt a connection to the first endpoint in the list. Each endpoint will be tried until we successfully establish a connection.
        auto endpoint = * endpoint_iterator;
        
        // Try to connect
        socket_->async_connect ( endpoint,
                                 boost::bind ( &asio_service_raw::handle_connect,
                                               this,
                                               boost::asio::placeholders::error, 
                                               ++endpoint_iterator ) );
    }
    else error_handler( err );
}

void asio_service_raw::handle_connect ( 
                                        const boost::system::error_code & err,
                                        boost::asio::ip::tcp::resolver::iterator endpoint_iterator
                                      )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_raw::handle_connect socket ptr null" );
    
    if ( !err )
    {
        /// WARNING - Make this time-out
        // The connection was successful. Send the buffer data
        boost::asio::async_write( *socket_.get(),
                                  request_,
                                  boost::bind ( &asio_service_raw::handle_write_request, 
                                                this,
                                                boost::asio::placeholders::error ) );
    }
    else if ( endpoint_iterator != boost::asio::ip::tcp::resolver::iterator() )
    {
        // The connection failed. Try the next endpoint in the list.
        socket_->close();
        auto endpoint = *endpoint_iterator;
        
        socket_->async_connect( endpoint,
                                boost::bind ( &asio_service_raw::handle_connect, 
                                              this,
                                              boost::asio::placeholders::error, 
                                              ++endpoint_iterator ) );
    }
    else error_handler( err );
}

void asio_service_raw::handle_write_request ( const boost::system::error_code & err )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_raw::handle_write_request socket ptr null" );
    
    if ( !err )
    {
        /// NOTE: Make this time-out, we don't want to wait forever for a reply!

        // Read until we get an </EOF!>
        boost::asio::async_read_until( *socket_.get(),
                                       response_, 
                                       "</EOF!>",
                                       boost::bind( &asio_service_raw::handle_read_response, 
                                                    this,
                                                    boost::asio::placeholders::error ) );
       
    }
    else error_handler( err );
}


void asio_service_raw::handle_read_response ( const boost::system::error_code & err )
{
    if ( !err )
    {
        // Set the operation as complete
        complete_ = true;
        
        // If a Callback is set - call it now
        if ( callback_ )
            callback_( response_ );
        
        // Else call this class's reply handler
        else
            handle_reply();

        // NOTE: Optional, close socket?
    }
    else error_handler( err );

}


}
}
