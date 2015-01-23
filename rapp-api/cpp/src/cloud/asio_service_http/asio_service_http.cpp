#include "asio_service_http.hpp"

namespace rapp {
namespace services {


asio_service_http::asio_service_http ( 
                                            const std::string & header,
                                            const std::string & post
                                         )
: header_ ( header ), 
  post_ ( post )
{ }
    
asio_service_http::asio_service_http (
                                            const std::string & header,
                                            const std::string & post,
                                            std::function<void( boost::asio::streambuf & )> callback
                                         )
: header_ ( header ), 
  post_ ( post ), 
  callback_ ( callback )
{ }

void asio_service_http::Schedule (
                                        boost::asio::ip::tcp::resolver::query & query,
                                        boost::asio::ip::tcp::resolver & resolver,
                                        boost::asio::io_service & io_service
                                    )
{
    socket_ = std::unique_ptr<boost::asio::ip::tcp::socket>( new boost::asio::ip::tcp::socket( io_service ) );
    std::ostream request_stream ( &request_ );
    request_stream << header_ << post_ << "\r\n";
    //std::string raw ( ( std::istreambuf_iterator<char>( &request_ ) ), std::istreambuf_iterator<char>() );

    resolver.async_resolve( query,
                            boost::bind( & asio_service_http::handle_resolve,
                                          this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::iterator ) );
}

void asio_service_http::handle_reply ( )
{
    /*
    std::istream is ( &response_ ); 
    std::string line;
    std::string reply;
    
    while ( std::getline( is, line ) )
        reply.append( line );

    std::cout << reply << std::endl;
    */
    
    std::string raw ( ( std::istreambuf_iterator<char>( &response_ ) ), std::istreambuf_iterator<char>() );
    std::cout << raw << std::endl;
}

void asio_service_http::error_handler ( const boost::system::error_code & error )
{
    std::cerr << error.message() << std::endl;
}

void asio_service_http::invalid_request ( const std::string message )
{
    std::cerr << message << std::endl;
}

void asio_service_http::handle_resolve ( 
                                            const boost::system::error_code & err,
                                            boost::asio::ip::tcp::resolver::iterator endpoint_iterator
                                         )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_http::handle_resolve socket ptr null" );
    
    if (!err)
    {
        // Attempt a connection to the first endpoint in the list. Each endpoint will be tried until we successfully establish a connection.
        auto endpoint = * endpoint_iterator;
        
        // Try to connect
        socket_->async_connect ( endpoint,
                                 boost::bind ( &asio_service_http::handle_connect,
                                               this,
                                               boost::asio::placeholders::error, 
                                               ++endpoint_iterator ) );
    }
    else error_handler( err );
}

void asio_service_http::handle_connect ( 
                                            const boost::system::error_code & err,
                                            boost::asio::ip::tcp::resolver::iterator endpoint_iterator
                                         )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_http::handle_connect socket ptr null" );
    
    if ( !err )
    {
        // The connection was successful. Send the request. NOTE: Response Will be Handled in callback ::handle_write_request
        boost::asio::async_write( *socket_.get(),
                                  request_,
                                  boost::bind ( &asio_service_http::handle_write_request, 
                                                this,
                                                boost::asio::placeholders::error ) );
    }
    else if ( endpoint_iterator != boost::asio::ip::tcp::resolver::iterator() )
    {
        // The connection failed. Try the next endpoint in the list.
        socket_->close();
        
        auto endpoint = *endpoint_iterator;
        
        socket_->async_connect( endpoint,
                                boost::bind ( &asio_service_http::handle_connect, 
                                              this,
                                              boost::asio::placeholders::error, 
                                              ++endpoint_iterator ) );
    }
    else error_handler( err );
}

void asio_service_http::handle_write_request ( const boost::system::error_code & err )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_http::handle_write_request socket ptr null" );
    
    if ( !err )
    {
        // Read the response status line - Callback handler is ::handle_read_status_line
        boost::asio::async_read_until( *socket_.get(),
                                       response_, 
                                       "\r\n",
                                       boost::bind( &asio_service_http::handle_read_status_line, 
                                                    this,
                                                    boost::asio::placeholders::error ) );
    }
    else error_handler( err );
}

void asio_service_http::handle_read_status_line ( const boost::system::error_code & err )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_http::handle_read_status_line socket ptr null" );
        
    if (!err)
    {
        // Check that response is OK.
        std::istream response_stream( &response_);
        std::string http_version;
        response_stream >> http_version;
        unsigned int status_code;
        response_stream >> status_code;
        std::string status_message;
        std::getline( response_stream, status_message );
        
        if ( !response_stream || http_version.substr(0, 5) != "HTTP/" )
        {
            invalid_request( "Invalid response" );
            return;
        }
        if ( status_code != 200 )
        {
            invalid_request( std::to_string( status_code ) );
            return;
        }

        // Read the response headers, which are terminated by a blank line. This is HTTP Protocol 1.0 & 1.1
        boost::asio::async_read_until( *socket_.get(),
                                       response_, 
                                       "\r\n\r\n",
                                       boost::bind ( &asio_service_http::handle_read_headers, 
                                                     this,
                                                     boost::asio::placeholders::error ) );
    }
    else error_handler( err );
}

void asio_service_http::handle_read_headers ( const boost::system::error_code & err )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_http::handle_read_headers socket ptr null" );
    
    if ( !err )
    {
        // Start reading remaining data until EOF.
        boost::asio::async_read ( *socket_.get(),
                                  response_,
                                  boost::asio::transfer_at_least( 1 ),
                                  boost::bind( &asio_service_http::handle_read_content, 
                                               this,
                                               boost::asio::placeholders::error ) );
        
        // Set the operation as complete
        complete_ = true;
        
        // If a Callback is set - call it now
        if ( callback_ )
            callback_( response_ );
        
        // Else call this class's reply handler
        else
            handle_reply();
    }
    else error_handler( err );
}

void asio_service_http::handle_read_content ( const boost::system::error_code & err )
{
    if ( !socket_ )
        throw std::runtime_error ( "asio_service_http::handle_read_content socket ptr null" );
    
    if ( !err )
    {
        // Continue reading remaining data until EOF - It reccursively calls its self - My guess is it uses the default async_read buffer length
        boost::asio::async_read ( *socket_.get(),
                                  response_,
                                  boost::asio::transfer_at_least( 1 ),
                                  boost::bind( &asio_service_http::handle_read_content, 
                                               this,
                                               boost::asio::placeholders::error ) );
    }
    else if ( err != boost::asio::error::eof )
        error_handler( err );
}


}
}
