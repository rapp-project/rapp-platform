#ifndef RAPP_SERVICE
#define RAPP_SERVICE
#include "Includes.ihh"

namespace rapp
{
namespace services
{
/**
 * @class service
 * @brief Base Class for RAPP Service Objects
 * @version 1
 * @date 21-December-2014
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 */

class service
{
  public:
      
      /// 
      Service (
                const std::string username,
                const std::string password
              );
      
      /// Asynchronous Run
      virtual void AsyncRun (  );

};
}
}
#endif