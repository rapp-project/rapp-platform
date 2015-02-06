#ifndef _RAPP_TCPServer_
#define _RAPP_TCPServer_
#include <rapp_server/TCPServer/Includes.hxx>

/**
 * 
 * 
 */

namespace rapp {
namespace cloud {

class TCPServer
{
  public:

    /// Create a TCPServer by passing as @param io_service
    TCPServer ( boost::asio::io_service & io_service )
    : acceptor_( io_service, tcp::endpoint( tcp::v4(), 9001 ) )
    {
        start_accept();
    }


  private:

    /**
     * Start accepting incoming connections
     */
    void start_accept()
    {   
        // Create a new connection pointer
        if ( auto new_connection = TCPConnection::create( acceptor_.get_io_service() ) )
        {
            // Give the TCP acceptor, that new connection pointer which will handle everything
            acceptor_.async_accept( new_connection->socket(), boost::bind( &TCPServer::handle_accept,
                                                                        this,
                                                                        new_connection,
                                                                        boost::asio::placeholders::error ) );
        }
        else
            throw std::runtime_error ( "TCPServer start_accept: could not alloc a new TCPConnection pointer" );
    }

    /**
     * Acceptor Error Handler
     * @param ?
     */
    void handle_accept (
                         boost::shared_ptr<TCPConnection> connection,
                         const boost::system::error_code& error
                       )
    {
        if ( !connection)
            throw std::runtime_error( "TCPServer handle_accept: null TCPConnection param" );
        
        if ( error )
            std::cerr << "TCPServer::handle_accept: " << error.message() << std::endl;

        connection->start();
        start_accept();
    }


    /// Boost asynchronous TCP Acceptor
    boost::asio::ip::tcp::acceptor acceptor_;
};

}
}

#endif