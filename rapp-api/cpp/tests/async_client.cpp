#include "../src/sockets/asio_service_client/asio_service_client.hpp"
#include "../src/sockets/service_controller/service_controller.hpp"
#include <boost/lexical_cast.hpp>
#include <thread>

int main ( int argc, char* argv[] )
{
    try
    {
        if ( argc != 3 )
        {
            std::cout << "Usage: async_client <server> <path>\n";
            std::cout << "Example:\n";
            std::cout << "  async_client localhost /service.php\n";
            return 1;
        }

        /* This class only contains the Resolver (URL address)
         * HOWEVER: by encapsulating io_service in asio_service_client, we are essentially blocking each and every service call
         * THEREFORE: We need to either group service calls, or thread them: 
         * http://codelever.com/blog/2013/06/18/using-boost-asio-io-service-in-another-thread/
         * 
         * Thus, creating a FIFO queue, which runs async io_services, one after the other (and not all together).
         * Furthermore, I should create a Cloud / Robot Service Queue Class. 
         * That class should contain a SINGLE io_service, or create many services, each for each service call.
         * However, all the service calls should be threaded, and asynchronous:
         *      1 - A 1st service may finish earlier than the last service invoked, therefore there is no need to block it
         *      2 - Concurrent service processing on the cloud does NOT require Linear/Stacked calls from the Robot.
         */
        rapp::services::service_controller ctrl;
        
        // WARNING this is a TEST - server & page must be safely encapsulated in RAPP::API
        const std::string server = boost::lexical_cast<std::string>( argv[1] );
        const std::string page = boost::lexical_cast<std::string>( argv[2] );
        
        const std::string post = "user=alex&pwd=qwepoi";
        std::string header = "POST " + page + " HTTP/1.1\r\n";
        header += "Host: " + server + "\r\n";
        header += "Content-Type: application/x-www-form-urlencoded\r\n";
        header += "Content-Length: " + boost::lexical_cast<std::string>( post.length() ) + "\r\n";
        header += "Connection: close\r\n\r\n";
        
        // with a callback => the callback will run once finished
        auto c1 = std::make_shared<rapp::services::asio_service_client>( header, post );
        auto c2 = std::make_shared<rapp::services::asio_service_client>( header, post );
        auto c3 = std::make_shared<rapp::services::asio_service_client>( header, post );
        auto c4 = std::make_shared<rapp::services::asio_service_client>( header, post, 
                                                                         [&]( boost::asio::streambuf & buffer )
                                                                         { 
                                                                            std::string raw ( ( std::istreambuf_iterator<char>( &buffer ) ), std::istreambuf_iterator<char>() );
                                                                            std::cout << raw << std::endl;
                                                                         } );

        /// Run all jobs together (asynchronous)
        auto jobs = { c1, c2, c3, c4 };
        ctrl.runJobs( jobs );
        
        /// Run each job, one at a time (serial)
        ctrl.runJob ( c1 );
        ctrl.runJob ( c2 );
        ctrl.runJob ( c3 );
        ctrl.runJob ( c4 );
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
    }

    return 0;
}