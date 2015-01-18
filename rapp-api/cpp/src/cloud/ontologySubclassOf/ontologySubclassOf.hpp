#ifndef RAPP_SERVICE_CLOUD_ONTOLOGY_SUBCLASS_OF
#define RAPP_SERVICE_CLOUD_ONTOLOGY_SUBCLASS_OF
#include "Includes.ihh"

namespace rapp {
namespace services {
namespace cloud {
    
/**
 * @class ontologySubclassOf
 * @brief Asynchronous Service which will request the Ontology Subclass of/for an Input
 * @version 0
 * @date 18-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * TODO Finish, TEST and Cleanup?
 * 
 * NOTE: If you decide to use std::vector<std::string> to store results, this class shoulb be enough.
 *       However, if you want class encapsulation, I can make a rapp::object::subclassOf, or, rapp::object::superclassOf
 *       And then parse the service JSON reply into such a vector.
 */

class ontologySubclassOf : public service    
{
  public:
      
      /**
       * Constructor
       * 
       * @param query is the entity for which we will try to acquire its Super-Ordinates or Sub-Ordinates (?)
       * @param callback is the functor that will receive the classes discovered
       * 
       * @note This is any asynchronous function!
       *       ontologySubclassOf::handler will execute on completion, and delegate results to callback functor.
       *       However, there is no promise on how long this will take, as processing times may vary!
       */
      ontologySubclassOf (
                            const std::string query
                            std::function< void( std::vector<std:string> ) > callback
                         );
    
      /**
       * Get the async_client as a Job for the service scheduler
       * @warning You have to schedule the execution, it is not done automatically!
       */
      std::shared_ptr<rapp::services::asio_service_client> Job ( ) const;
      
  private:
      
      /// Parse @param buffer received from the socket, into a vector of faces
      void handle ( boost::asio::streambuf & buffer );
      
      
      
      /// The callback called upon completion of receiving the detected faces
      std::function< void( std::vector<std:string> classes ) > callback__;
      
      /// The ASIO Client Socket used internally
      std::shared_ptr<rapp::services::asio_service_client> client__;
};
  
    
}
}
}

#endif