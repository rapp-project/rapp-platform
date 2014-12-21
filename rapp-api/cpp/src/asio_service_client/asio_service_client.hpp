#ifndef RAPP_ASIO_SERVICE_CLIENT
#define RAPP_ASIO_SERVICE_CLIENT
#include "Includes.ihh"

namespace rapp
{
namespace services
{

/**
 * @class asio_service_client
 * @brief base class for asynchronous http websockets used for connecting to cloud services
 * @version 1
 * @date 20-December-2014
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * TODO: Enable Time-out timers for a client connection ???
 * NOTE: This is subjective, some processing may take quite some time! It needs experimentation and a unique timeout per service
 * 
 * HTTP Protocol @see http://www.jmarshall.com/easy/http/#postmethod
 * Original Source @see http://www.boost.org/doc/libs/1_41_0/doc/html/boost_asio/example/http/client/async_client.cpp
 * 
 * @note this class will NOT Block the socket or process, and will not accept streams, only text data
 * @note this class will not work with SSL/TLS sockets
 */
class asio_service_client
{
  public:
    

    /**
     * Construct the client by passing:
     * @param io_service a boost service Async IO scheduler
     * @param query is a resolved URL using boost IP TCP resolution
     * @param header is the actual HTTP Header crafted accordingly
     * @param post is the actual $_POST field following the HTTP Header
     */
    asio_service_client ( 
                            boost::asio::io_service & io_service,
                            boost::asio::ip::tcp::resolver::query & query,
                            const std::string & header,
                            const std::string & post
                        );

    // TODO
    asio_service_client ( 
                        boost::asio::io_service & io_service,
                        boost::asio::ip::tcp::resolver::query & query,
                        const std::string & header,
                        const std::string & post,
                        std::function<void( boost::asio::streambuf & )> callback
                    );

    /**
     * Handle the Reply
     * @note you have to override this method if inheriting
     */
    virtual void handle_reply ( );
    
    /**
     * Handle an Error
     * @param error is the raised error from the client
     */
    virtual void error_handler ( const boost::system::error_code & error );

    /**
     * Invalid Query Handler - different from Error Handler, we have a response which states our query was invalid
     * @param message is the message received from the service
     */
    virtual void invalid_request ( const std::string message );

    /**
     * Has this client completed its operation
     * @return true or false
     */
    bool complete ( ) const;
    
    
  private:
    
    /** 
     * Callback for Handling Address Resolution
     * @param err is a possible error
     * @param endpoint_iterator is boost's hostname address handler
     */
    void handle_resolve ( 
                            const boost::system::error_code & err,
                            boost::asio::ip::tcp::resolver::iterator endpoint_iterator
                        );

    /**
     * Callback for Handling Connection Events
     * @param err is a possible error
     * @param endpoint_iterator is boosts' hostname address handler
     */
    void handle_connect ( 
                            const boost::system::error_code & err,
                            boost::asio::ip::tcp::resolver::iterator endpoint_iterator
                        );

    /**
     * Callback for handling request and waiting for response
     * @param err is a possible error
     */
    void handle_write_request ( const boost::system::error_code & err );
    
    /**
     * Callback for handling Response Data
     * @param err is a possible error message
     */
    void handle_read_status_line ( const boost::system::error_code & err );

    /**
     * Callback for Handling Headers
     * @param err is a possible error message
     */
    void handle_read_headers ( const boost::system::error_code & err );
    
    /**
     * Callback for Handling Actual Data Contents
     * @param err is a possible error message
     */
    void handle_read_content ( const boost::system::error_code & err );
    
    
    
    /// Resolves URL/URIs
    boost::asio::ip::tcp::resolver resolver_;
    
    /// Actual Socket
    boost::asio::ip::tcp::socket socket_;
    
    /// Request Container
    boost::asio::streambuf request_;
    
    /// Response Container
    boost::asio::streambuf response_;
    
    /// Operation complete?
    bool complete_ = false;
    
    /// Optional Callback Handler
    std::function<void( boost::asio::streambuf & )> callback_;
};
}
}

#endif