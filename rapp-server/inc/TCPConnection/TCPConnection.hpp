#ifndef _RAPP_TCPConnection_
#define _RAPP_TCPConnection_
#include "Includes.hxx"

/// TODO: Make this a polymorphic class, and then use inherited classes for handling "respond"
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
        /// WARNING - Enable time-outs, we don't want to wait on hang-ups, time-outs or errors
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
        /*
         * NOTE - Ideally, at this point, you want some kind of handler, to check the HTTP Header,
         *        and see what kind of file we've just received. Is it Content-type: image/jpg, or something else?
         * 
         * BUG - Something is wrong, because there are way more bytes than there should be.
         *       I count 38119 bytes in the copy_of_picture, whilst the original one is only 21173 bytes.
         */
        
        std::vector<byte> imagebytes;
        
        // Search for the `<IMG!>` delimiter - then copy from that position, and up to the position of <EOF!>
        for ( unsigned int i = 0; i < bytearray_.size(); i++ )
        {
            if ( (i + 5) < (bytearray_.size()-7) )
            {
                if ( std::string( &bytearray_[i], 5 ) == "<IMG>" )
                {
                    std::copy( bytearray_.begin() + i + 5,
                               bytearray_.end() - 7,
                               std::back_inserter( imagebytes) );
                    break;
                }
            }   
        }
        
        std::cout << "Image bytes: " << imagebytes.size() << std::endl;
        std::ofstream os ( "copy_of_picture.jpg", std::ios::out | std::ofstream::binary );
        std::copy( imagebytes.begin(), imagebytes.end(), std::ostreambuf_iterator<char>( os ) );
        
        
        /*
         * If you want to extract the binary data, then you have to remove the HTTP Header and the two returns \r\n
         * Alternatively, you can use a starting TAG, such as <IMG!> to find where the binary data starts.
         * DO NOT CAST the bytearray into a std::string if you're manipulating images or audio, this will not work,
         * unless the data is base64 encoded.
         * 
         * From there, you simply iterate until you find the <EOF!>
         * 
         * If you do not like the Delimiter approach, then you can read the Header, using boost::asio::async_read
         * This uses the MTU of the socket (in UNIX its 1500) so, you can read the header and some more.
         * From there, calculate how many bytes you have to read, and how many you currently have read,
         * and you keepr reading until you have the bytes you want.
         */
        
        // NOTE: We are replying with HTTP, thus we need a header.
        //reply_ = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 8\r\nConnection: close\r\n\r\nBye!\r\n";

        reply_ = "OK, Thanks! Bye</EOF!>";
         
        //std::cout << reply_ << std::endl;

        // write back the response - WARNING upon replying, socket is CLOSED - WARNING Force Timeout
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
            std::cout << "Processing bytes in buffer" << std::endl;
            std::istream is ( &buffer_ );
            
            std::copy(  std::istreambuf_iterator<byte>( is ),
                        std::istreambuf_iterator<byte>(),
                        std::back_inserter( bytearray_ ) );
            
            //std::cout << "Bytes read: " << bytes_transferred << std::endl;
            
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
