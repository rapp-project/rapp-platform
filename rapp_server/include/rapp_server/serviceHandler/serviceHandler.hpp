#ifndef _RAPP_SERVICEHANDLER_
#define _RAPP_SERVICEHANDLER_

#include <rapp_server/serviceHandler/Includes.ihh>

/**
 * @class serviceHandler
 * @brief abstract base class for all service handlers
 * @version 1
 * @date 6-February-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 */

namespace rapp {
namespace cloud {

class serviceHandler
{
  public:
   
   typedef char byte;
    
   /**
    * @brief Pure virtual function that processes the request.
    * @param bytearray is the raw data from the socket buffer
    * @return a string reply
    * @note if the return type doesn't suit us, create another pure virtual method?
    */
   virtual std::string process (  const std::vector<byte> & bytearray ) = 0;

   
   virtual ~serviceHandler() { };

};

}
}

#endif
