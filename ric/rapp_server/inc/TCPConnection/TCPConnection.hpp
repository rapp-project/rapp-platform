#ifndef _RAPP_TCPConnection_
#define _RAPP_TCPConnection_
#include "Includes.hxx"

class TCPConnection : public boost::enable_shared_from_this<TCPConnection>
{
  public:
    
    
    typedef char byte;

    /// Create a new TCP Connection
    static boost::shared_ptr<TCPConnection> create ( boost::asio::io_service & io_service )
    {
        return boost::shared_ptr<TCPConnection>( new TCPConnection ( io_service ) );
    }

    
    tcp::socket& socket()
    {
        return socket_;
    }
    
    /** 
     * Start Reading Async Until </EOF!> is encountered
     * 
     * @warning This is a delimiter based transmission.
     *          Alternatively, we can read the HEADER Content-Length,
     *          and instead of async_read_until, try async_read_some ( bytes )
     */
    void start ( )
    {
        boost::asio::async_read_until( socket_,
                                       buffer_,
                                       "</EOF!>",
                                       boost::bind( &TCPConnection::handle_read,
                                                    shared_from_this(),
                                                    boost::asio::placeholders::error,
                                                    boost::asio::placeholders::bytes_transferred
                                                  ) );
    }
    

    /**
     * NOTE - This is where you can delegate Message processing
     *        You can do this, either by creating a singleton class which handles the data in here
     *        Or, you can create a class handler, which is unique to each message, and construct the message in here
     *        Ideally, you may want a threaded or multi process (fork+exec) handler here which will send
     *        messages to ROS.
     */
    void respond ( )
    {
        // I am storing the messages in a bytearray on purpose! We do not know if its string, image or audio
        for ( const auto & byte : bytearray_ ) std::cout << byte;
        std::cout << std::endl;

        // NOTE: We are replying with HTTP, thus we need a header.
        reply_ = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 8\r\nConnection: close\r\n\r\nBye!\r\n";
         
        std::cout << reply_ << std::endl;

        // write back the response - WARNING upon replying, socket is CLOSED
        boost::asio::async_write( socket_,
                                  boost::asio::buffer( reply_ ),
                                  boost::bind (
                                               &TCPConnection::handle_write,
                                               shared_from_this(),
                                               boost::asio::placeholders::error,
                                               boost::asio::placeholders::bytes_transferred
                                              ) );
    }

  protected:


    /// Protected Constructor
    TCPConnection ( boost::asio::io_service& io_service ) : socket_( io_service )
    {
        reply_ = "";
        bytes_sent_ = 0;
    }

    /** 
     * Handle asynchronous read events
     * @warning depending on the size of data transferred this may be called once, or many times
     * @note This will work for strings - for raw data, we need to use 
     */
    void handle_read (
                        const boost::system::error_code & error,
                        size_t bytes_transferred
                     )
    {
        if ( error )
        {
            std::cerr << "TCPConnection handle_read: Error: " << error.message() << std::endl;
            std::cerr << "TCPConnection handle_write: Bytes read: " << bytes_transferred << std::endl;
        }
        else if ( !error )
        {
            std::istream is ( &buffer_ );
            byte temp;
            
            while ( is >> temp )
                bytearray_.push_back( temp );

            // Delegate response
            respond ( );
        }
    }

    // Handle asynchronous write events
    void handle_write (
                        const boost::system::error_code & error,
                        size_t bytes_transferred
                      )
    {
        if ( error )
            std::cerr << "TCPConnection handle_write: " << error.message() << std::endl;

        else
            bytes_sent_ += bytes_transferred;

        // Close the socket once we've written back a reply
        if ( !error && bytes_sent_ == reply_.size() )
            socket_.close();
    }



    /// Connecting Socket
    tcp::socket socket_;

    /// Received Data
    std::vector<byte> bytearray_;

    /// Incoming Message Buffer
    boost::asio::streambuf buffer_;

    /// Reply to Client
    std::string reply_;

    /// Bytes Sent to Client
    std::size_t bytes_sent_ = 0;
};

#endif