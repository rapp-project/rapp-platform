#include "../src/asio_service_client/asio_service_client.hpp"
#include "../src/service_controller/service_controller.hpp"
#include <boost/lexical_cast.hpp>

int main(int argc, char* argv[])
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

        rapp::services::service_controller ctrl;
        
        const std::string server = boost::lexical_cast<std::string>( argv[1] );
        const std::string page = boost::lexical_cast<std::string>( argv[2] );
        
        const std::string post = "user=alex&pwd=qwepoi";
        std::string header = "POST " + page + " HTTP/1.1\r\n";
        header += "Host: " + server + "\r\n";
        header += "Content-Type: application/x-www-form-urlencoded\r\n";
        header += "Content-Length: " + boost::lexical_cast<std::string>( post.length() ) + "\r\n";
        header += "Connection: close\r\n\r\n";
        
        // without a callback => asio_service_client::handle_reply will run once finished
        //rapp::services::asio_service_client c ( ctrl.Scheduler(), ctrl.Resolver(), header, post );
        
        // with a callback => the callback will run once finished
        rapp::services::asio_service_client c ( ctrl.Scheduler(), ctrl.Resolver(), header, post,
                                                [&]( boost::asio::streambuf & buffer )
                                                { 
                                                    std::string raw ( ( std::istreambuf_iterator<char>( &buffer ) ), 
                                                                                       std::istreambuf_iterator<char>() );
                                                    std::cout << raw << std::endl;
                                                } );
        
        ctrl.Scheduler().run();
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
    }

    return 0;
}