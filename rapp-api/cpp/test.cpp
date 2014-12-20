#include "src/asio_service_client.hpp"

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

        // WARNING: The boost::io_service should be encapsulated somehow into the rapp::api - Either a Global Variable or a Singleton?
        boost::asio::io_service io_service;
        
        // THIS Is JUST A TEST
        rapp::services::asio_service_client c(io_service, argv[1], argv[2]);
        
        // We may hide this by using a namespace global var, or by wrapping in an object, to which the user has control over
        io_service.run();
        
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
    }

    return 0;
}