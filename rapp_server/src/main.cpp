#include <rapp_server/TCPServer/TCPServer.hpp>

boost::asio::io_service io_service;

int main ( )
{
    TCPServer server( io_service );
    io_service.run();

    return 0;
}
