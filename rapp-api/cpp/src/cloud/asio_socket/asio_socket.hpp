#ifndef RAPP_ASIO_SOCKET_
#define RAPP_ASIO_SOCKET_
#include "Includes.ihh"

namespace rapp
{
namespace services
{

/**
 * @brief Abstract Base ASIO Socket class
 * @note Use for passing around the service controller, various types of sockets
 */
class asio_socket
{
  public:


    /** 
     * Schedule this client as a job for execution using
     * @param query defines the actual URL/URI
     * @param resolver is the URL/URI resolver reference
     * @param io_service is the service queue on which this job will be scheduled to run
     */
    virtual void Schedule ( 
                             boost::asio::ip::tcp::resolver::query & ,
                             boost::asio::ip::tcp::resolver & ,
                             boost::asio::io_service &
                          ) = 0;
    
  protected:  
      
    /**
     * Handle the Reply
     * @note you have to override this method if inheriting
     */
    void handle_reply ( );
    
};
}
}

#endif
