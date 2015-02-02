#ifndef _RAPP_TCPConnection_
#define _RAPP_TCPConnection_
#include <rapp_server/TCPConnection/Includes.hxx>

class TCPConnection : public boost::enable_shared_from_this<TCPConnection>
{
  public:
    
    
    typedef char byte;

    /// Create a new TCP Connection
    static boost::shared_ptr<TCPConnection> create ( boost::asio::io_service & io_service )
    {
        return boost::shared_ptr<TCPConnection>( new TCPConnection ( io_service ) );
    }

    /// @Return socket
    tcp::socket& socket()
    {
        return socket_;
    }
    
    /** 
     * Start Reading Async Until </EOF!> is encountered
     * @warning This is a delimiter based transmission.
     *          Alternatively, we can read the HEADER Content-Length,
     *          and instead of async_read_until, try async_read_some ( bytes )
     */
    void start ( )
    {
        // 5 second time-out
        timer_.expires_from_now( boost::posix_time::seconds( 5 ) );

        // WARNING - Enable time-outs, we don't want to wait on hang-ups, time-outs or errors
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
    virtual void process ( )
    {           
        std::vector<byte> imagebytes;
        
        // Search for the `<IMG!>` delimiter - then copy from that position, and up to the position of <EOF!>
        for ( unsigned int i = 0; i < bytearray_.size(); i++ )
        {
            if ( (i + 5) < (bytearray_.size()-7) )
            {
                // NOTE: In order to avoid copying 5 bytes into string, maybe test first char is `<`
                if ( std::string( &bytearray_[i], 5 ) == "<DAT>" )
                {
                    std::copy( bytearray_.begin() + i + 5,
                               bytearray_.end() - 7,
                               std::back_inserter( imagebytes) );
                    break; // We got what we wanted, break out of the loop
                }
            }   
        }
        
        // Copy Image Bytes to a file on Disk
        std::cout << "Image bytes: " << imagebytes.size() << std::endl;
        std::ofstream os ( "copy_of_picture.jpg", std::ios::out | std::ofstream::binary );
        std::copy( imagebytes.begin(), imagebytes.end(), std::ostreambuf_iterator<char>( os ) );
                
        // Always reply with an </EOF!> because the C++ client side expects this delimiter
        //reply_ = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 8\r\nConnection: close\r\n\r\nBye!\r\n";
        reply_ = "Bye</EOF!>";
        //std::cout << reply_ << std::endl;

        // 5 second time-out 
        timer_.expires_from_now( boost::posix_time::seconds( 5 ) );

        // write back the response - socket closes - timeout applied
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
    TCPConnection ( boost::asio::io_service& io_service ) 
    : socket_ ( io_service ),
      timer_ ( io_service )
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
            
            std::copy(  std::istreambuf_iterator<byte>( is ),
                        std::istreambuf_iterator<byte>(),
                        std::back_inserter( bytearray_ ) );
            
            // Delegate buffer processing
            process ( );
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
        {
            socket_.close();
            std::cout << "Connection closed" << std::endl;
        }
    }



    /// Connecting Socket
    boost::asio::ip::tcp::socket socket_;

    /// Incoming Message Buffer
    boost::asio::streambuf buffer_;

    /// Timer for forcing timeouts
    boost::asio::deadline_timer timer_;

    /// Received Data
    std::vector<byte> bytearray_;

    /// Connection was stopped
    bool stopped_ = false;

    /// Reply to Client
    std::string reply_;

    /// Bytes Sent to Client
    std::size_t bytes_sent_ = 0;
};

#endif
