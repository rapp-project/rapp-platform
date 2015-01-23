#ifndef RAPP_ASIO_SERVICE_HTTP
#define RAPP_ASIO_SERVICE_HTTP
#include "Includes.ihh"

namespace rapp
{
namespace services
{

/**
 * @class asio_service_http
 * @brief base class for asynchronous http websockets used for connecting to cloud services
 * @version 4
 * @date 5-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * @see http://www.jmarshall.com/easy/http/#postmethod for HTTP Protocol details
 * @warning this class does not support SSL/TLS sockets
 * 
 * TODO: Enable Time-out timers for a client connection ???
 * NOTE: This is subjective, some processing may take quite some time! 
 *       It needs experimentation and a unique timeout per service
 */
class asio_service_http : public asio_socket
{
  public:

    /**
     * Construct the async client. 
     */
    asio_service_http (
                          const std::string & header,
                          const std::string & post
                        );
    
    /**
     * @param header is the actual HTTP Header crafted accordingly
     * @param post is the actual $_POST field following the HTTP Header
     * @param callback is a lamda or function pointer with the specific signature, that is invoked upon reply acquisition
     */
    asio_service_http (
                           const std::string & header,
                           const std::string & post,
                           std::function<void( boost::asio::streambuf & )> callback
                        );

    /** 
     * Schedule this client as a job for execution using
     * @param query defines the actual URL/URI
     * @param resolver is the URL/URI resolver reference
     * @param io_service is the service queue on which this job will be scheduled to run
     */
    void Schedule ( 
                    boost::asio::ip::tcp::resolver::query & query,
                    boost::asio::ip::tcp::resolver & resolver,
                    boost::asio::io_service & io_service
                  );
    
  protected:  
      
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
    

    
    /// Header that will be used
    std::string header_;
    
    /// Actual post Data
    std::string post_;
    
    /// Optional Callback Handler
    std::function<void( boost::asio::streambuf & )> callback_;
    
    /// Actual Socket
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
    
    /// Request Container
    boost::asio::streambuf request_;
    
    /// Response Container
    boost::asio::streambuf response_;
    
    /// Operation complete?
    bool complete_ = false;
};
}
}

#endif
