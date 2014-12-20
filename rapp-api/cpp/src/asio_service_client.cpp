#include "asio_service_client.hpp"

// TODO: We Need a Time-out if Server is Not responding
namespace rapp {
namespace services {

asio_service_client::asio_service_client ( 
                                            boost::asio::io_service & io_service,
                                            const std::string & server, 
                                            const std::string & path
                                        )
: resolver_( io_service ), socket_( io_service )
{        
    std::ostream request_stream( &request_ );
    
    // WARNING: Path needs a trailing forwardslash, e.g.: "/service.php"
    request_stream << "POST " << path << " HTTP/1.1\r\n";
    request_stream << "Host: " << server << "\r\n";
    request_stream << "Accept: */*\r\n";
    request_stream << "User-Agent: RAPP-API\r\n";
    request_stream << "Connection: close\r\n\r\n";

    // NOTE: using the same tcp::resolver::query may be an overkill if many service objects are used. 
    // TODO: Maybe I should re-use it as a parameter to the constructor
    tcp::resolver::query query ( server, "http", tcp::resolver::query::canonical_name );
    resolver_.async_resolve( query,
                             boost::bind( &asio_service_client::handle_resolve, 
                                          this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::iterator ) );
}

std::string asio_service_client::reply_handler ( )
{
    //std::string raw ( ( std::istreambuf_iterator<char>( &response_ ) ), std::istreambuf_iterator<char>() );
    std::istream is ( &response_ ); 
    std::string line;
    std::string reply;
    unsigned int i = 0;
    
    while ( std::getline( is, line ) )
    {
        if ( i > 5 )    // CAUTION WARNING - This assumes that up to line #5 we have header stuff
        {
           reply.append( line );
        }
        i++;
        /* 
         * NOTE: I could use REGEX to find specific Header keywords (Date, Server, Content-Length, Connection, Content-Type)
         *       However, I don't really case, as this method should be overriden by each inheriting class.
         * 
         * NOTE: Another way, is to use std::search or other substring finding methods.
         */
    }
    
    std::cout << "REPLY: " << reply << std::endl;
    
    return reply;
}

void asio_service_client::error_handler ( const boost::system::error_code & error )
{
    std::cerr << error.message() << std::endl;
}

void asio_service_client::invalid_query ( std::string message )
{
    std::cerr << message << std::endl;
}

void asio_service_client::handle_resolve ( 
                                            const boost::system::error_code & err,
                                            tcp::resolver::iterator endpoint_iterator
                                         )
{
    if (!err)
    {
        // Attempt a connection to the first endpoint in the list. Each endpoint will be tried until we successfully establish a connection.
        tcp::endpoint endpoint = * endpoint_iterator;
        
        // Try to connect
        socket_.async_connect ( endpoint,
                                boost::bind ( &asio_service_client::handle_connect,
                                              this,
                                              boost::asio::placeholders::error, 
                                              ++endpoint_iterator ) );
    }
    else error_handler( err );
}

void asio_service_client::handle_connect ( 
                                            const boost::system::error_code & err,
                                            tcp::resolver::iterator endpoint_iterator
                                         )
{
    if ( !err )
    {
        // The connection was successful. Send the request. NOTE: Response Will be Handled in callback ::handle_write_request
        boost::asio::async_write( socket_, 
                                  request_,
                                  boost::bind ( &asio_service_client::handle_write_request, 
                                                this,
                                                boost::asio::placeholders::error ) );
    }
    else if ( endpoint_iterator != tcp::resolver::iterator() )
    {
        // The connection failed. Try the next endpoint in the list.
        socket_.close();
        tcp::endpoint endpoint = *endpoint_iterator;
        socket_.async_connect( endpoint,
                               boost::bind ( &asio_service_client::handle_connect, 
                                             this,
                                             boost::asio::placeholders::error, 
                                             ++endpoint_iterator ) );
    }
    else error_handler( err );
}

void asio_service_client::handle_write_request ( const boost::system::error_code & err )
{
    if ( !err )
    {
        // Read the response status line - Callback handler is ::handle_read_status_line
        boost::asio::async_read_until( socket_, 
                                       response_, 
                                       "\r\n",
                                       boost::bind( &asio_service_client::handle_read_status_line, 
                                                    this,
                                                    boost::asio::placeholders::error ) );
    }
    else error_handler( err );
}

void asio_service_client::handle_read_status_line ( const boost::system::error_code & err )
{
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
            invalid_query( "Invalid response" );
            return;
        }
        if ( status_code != 200 )
        {
            invalid_query( std::to_string( status_code ) );
            return;
        }

        // Read the response headers, which are terminated by a blank line. This is HTTP Protocol
        boost::asio::async_read_until( socket_, 
                                       response_, 
                                       "\r\n\r\n",
                                       boost::bind ( &asio_service_client::handle_read_headers, 
                                                     this,
                                                     boost::asio::placeholders::error ) );
    }
    else error_handler( err );
}

void asio_service_client::handle_read_headers ( const boost::system::error_code & err )
{
    if (!err)
    {
        // Start reading remaining data until EOF.
        boost::asio::async_read ( socket_, 
                                  response_,
                                  boost::asio::transfer_at_least( 1 ),
                                  boost::bind( &asio_service_client::handle_read_content, 
                                               this,
                                               boost::asio::placeholders::error ) );
        
        // Finally call the reply handler
        reply_handler( );
    }
    else error_handler( err );
}

void asio_service_client::handle_read_content ( const boost::system::error_code & err )
{
    if ( !err )
    {
        // Continue reading remaining data until EOF - WARNING is this the best way for fast reading ? NOTE It reccursively calls its self - NOTE: My guess is it uses the default async_read buffer length
        boost::asio::async_read ( socket_, 
                                  response_,
                                  boost::asio::transfer_at_least( 1 ),
                                  boost::bind( &asio_service_client::handle_read_content, 
                                               this,
                                               boost::asio::placeholders::error ) );
    }
    else if ( err != boost::asio::error::eof )
        error_handler( err );
}
}
}