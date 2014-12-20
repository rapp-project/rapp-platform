#ifndef RAPP_ASIO_SERVICE_CLIENT
#define RAPP_ASIO_SERVICE_CLIENT
#include "Includes.ihh"
using boost::asio::ip::tcp;
namespace rapp {
namespace services {

/**
 * @class asio_service_client
 * @brief base class for asynchronous http websockets used for connecting to cloud services
 * @version 1
 * @date 20-December-2014
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * TODO: We Need POST Headers that send a JSON query
 * TODO: Enable Time-out timers for a client connection ?
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
     * @param io_service a boost service ref object
     * @param server the actual URL or IP address (we assume Unix: port 80)
     * NOTE: We need to pass as an optional parameter the POST Data
     */
    asio_service_client ( 
                            boost::asio::io_service & io_service,
                            const std::string & server, 
                            const std::string & path
                        );
    
    /**
     * Handle a Reply - TODO call this at any moment an error is detected
     * @note you may wanna override this method if not acquiring text
     */
    virtual std::string reply_handler ( );
    
    /**
     * Handle an Error - TODO call this only once handle_read_content has finished.
     * @param error is the raised error from the client
     */
    virtual void error_handler ( const boost::system::error_code & error );

    /**
     * Invalid Query Handler - this differs from Error Handler, as we receiveda response which states our query was invalid
     * @param error is the message received from the service
     */
    virtual void invalid_query ( std::string message );

    
  private:
    
    /** 
     * Callback for Handling Address Resolution
     * @param err is a possible error
     * @param endpoint_iterator is boost's address iterator
     */
    void handle_resolve ( 
                            const boost::system::error_code & err,
                            tcp::resolver::iterator endpoint_iterator
                        );

    /**
     * Callback for Handling Connection Events
     * @param err is a possible error
     * @param endpoint_iterator is boosts' address handler
     */
    void handle_connect ( 
                            const boost::system::error_code & err,
                            tcp::resolver::iterator endpoint_iterator
                        );

    /**
     * Callback for handling request and waiting for response - Async Read Until "\r\n" 
     * WARNING is this the delimiter we want?
     * @param err is a possible error
     */
    void handle_write_request ( const boost::system::error_code & err );
    
    /**
     * Callback for handling Response Data
     * TODO: Proper Error Handling?
     * @param err is a possible error message
     */
    void handle_read_status_line ( const boost::system::error_code & err );

    /**
     * Callback for Handling Headers
     * @param err is a possible error message
     * NOTE: Do we really care about the Headers? - NO WE DO NOT unless there is an error in them
     */
    void handle_read_headers ( const boost::system::error_code & err );
    
    /**
     * Callback for Handling Actual Data Contents
     * @param err is a possible error message
     */
    void handle_read_content ( const boost::system::error_code & err );
    
    
    
    /// Resolves URL/URIs
    tcp::resolver resolver_;
    
    /// Actual Socket
    tcp::socket socket_;
    
    /// Request Container
    boost::asio::streambuf request_;
    
    /// Response Container
    boost::asio::streambuf response_;
};
}
}

#endif