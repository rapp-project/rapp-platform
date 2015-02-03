#ifndef _PROCESSOR_FACTORY_
#define _PROCESSOR_FACTORY_

#include <rapp_server/TCPConnection/rapp_service_processors/face_detection_processor.hpp>

// All the existent rapp processors
enum RappProcessors
{
  FACE_DETECTION_PROC
};

/**
 * @class RappProcessorFactory
 * @brief Provides the necessary types of processors
 */
class RappProcessorFactory
{
  public:
   /**
    * @brief Returns the correct processor
    * @param p [RappProcessors] Type of processor wanted
    * @return pointer of the base processor
    */
   static BaseRappConnectionProcessor* getProcessor(RappProcessors p)
   {
     BaseRappConnectionProcessor* proc = NULL;
     switch(p)
     {
       case FACE_DETECTION_PROC:
         proc = new FaceDetectionProcessor; 
         break;

       default: ;
         //throw exception
     } 
     return proc;
   }

};


#endif
