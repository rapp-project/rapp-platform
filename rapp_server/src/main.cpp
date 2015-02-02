#include <rapp_server/TCPServer/TCPServer.hpp>

boost::asio::io_service io_service;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "rapp_server_node");
  TCPServer server( io_service );
  io_service.run();

  return 0;
}
