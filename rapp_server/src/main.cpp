#include <rapp_server/TCPServer/TCPServer.hpp>
#include <ros_service_invoker/ros_service_factory.hpp>

boost::asio::io_service io_service;

int main (int argc, char** argv)
{
  /**
   * @date 6-February-2015
   * @author Alex Gkiokas
   * 
   * NOTE - You are now running a single server instance on Port 9001.
   *        TCPServer uses by default, TCPConnection.
   *        
   *        There are 2 options:
   * 
   *            1 - Run a single Server, and let the server deal with different service requests. Simple, but can get HUGE if
   *                we end up having too many services (30+)
   * 
   *            2 - Run different Servers here, one for each service request. make TCPServer a Template class, which
   *                uses as argument, the TCPConnection type. For each service Handler, make a new class that inherits TCPConnection.
   *                Complex at first, but will keep the namespace and code very clean, as we keep each handler seperate from TCP classes.
   *                Additionally, if each server listens and runs as a different Port/Thread, this will provide extra robustness
   * 
   * NOTE - A Server, and a service in Boost are two different things. We can use many services for many servers
   *        or many servers with one service.
   */
    
  ros::init( argc, argv, "rapp_server_node" );
  
  rapp::cloud::TCPServer server ( io_service );
  io_service.run();

  return 0;
}