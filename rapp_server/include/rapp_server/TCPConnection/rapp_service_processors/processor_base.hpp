#ifndef _BASE_PROCESSOR_
#define _BASE_PROCESSOR_

#include <rapp_server/TCPConnection/Includes.hxx>

/**
 * @class BaseRappConnectionProcessor
 * @brief Provides the base for every processor to be called from a
 * TCP connection
 */
class BaseRappConnectionProcessor
{
  public:
   /**
    * @brief Pure virtual function that processes the request.
    * @param bytearray [std::vector<char>&] Bytearray containing the request
    * @param response [std::string&] Must contain the processor's response
    */
   virtual void process(const std::vector<char>& bytearray, 
     std::string& response) = 0; 

   // Virtual destructor as class is abstract
   virtual ~BaseRappConnectionProcessor(){};

};

#endif
