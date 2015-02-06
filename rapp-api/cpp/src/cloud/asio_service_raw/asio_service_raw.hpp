#ifndef RAPP_ASIO_SERVICE_RAW
#define RAPP_ASIO_SERVICE_RAW
#include "Includes.ihh"

namespace rapp {
namespace services {

/**
 * @class asio_service_raw
 * @brief base class for asynchronous sockets used for connecting to cloud services
 * @version 2
 * @date 6-February-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * @warning this class does not support SSL/TLS sockets
 */
class asio_service_raw : public asio_socket
{
  public:
      
    /**
     * @brief Construct an Asynchronous Socket Client without HTTP stuff, only XML delimiters
     * @param bytearray is a vector which contains raw byte (char) values
     */
    asio_service_raw ( const std::vector<rapp::types::byte> & bytearray );
    
    /**
     * @brief Construct an Asynchronous Socket Client without HTTP stuff, only XML delimiters
     * @param bytearray is a vector which contains raw byte (char) values
     * @param callback is a lamda or function pointer with the specific signature, that is invoked upon reply acquisition
     */
    asio_service_raw (
                       const std::vector<rapp::types::byte> & bytearray,
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
     * Callback for handling sending request and waiting for a response
     * @param err is a possible error
     */
    void handle_write_request ( const boost::system::error_code & err );
    

    /**
     * Callback for handling reading the reponse from the server
     */
    void handle_read_response ( const boost::system::error_code & err );


    /// Actual bytestream that will be sent over the socket - @note: be careful not to consume it prematurely!
    std::vector<rapp::types::byte> bytes_; 
    
    /// Optional Callback Handler
    std::function<void( boost::asio::streambuf & )> callback_;
    
    /// Actual Socket
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
    
    /// Request buffer stream
    boost::asio::streambuf request_;
    
    /// Response buffer stream
    boost::asio::streambuf response_;
    
    /// Operation complete?
    bool complete_ = false;
};
}
}

#endif
