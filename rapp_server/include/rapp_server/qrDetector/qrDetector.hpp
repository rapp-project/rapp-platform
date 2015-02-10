#ifndef _RAPP_QR_DETECTOR_
#define _RAPP_QR_DETECTOR_

#include <rapp_server/qrDetector/Includes.hxx>

namespace rapp {
namespace cloud {

class qrDetector: public serviceHandler
{
  public:
    
    typedef char byte;
      
    std::string process ( const std::vector<byte> & bytearray )
    {
      std::vector<byte> imagebytes;
      std::string response;

      // Search for the `<IMG!>` delimiter - then copy from that position, 
      // and up to the position of <EOF!>
      for ( unsigned int i = 0; i < bytearray.size(); i++ )
      {
        if ( (i + 5) < ( bytearray.size()-7 ) )
        {
          // NOTE: In order to avoid copying 5 bytes into string, maybe test  first char is `<`?
          if ( std::string( &bytearray[i], 5 ) == "<IMG>" )    // Find <IMG>
          {
            std::copy ( bytearray.begin() + i + 5,             // length of <IMG>
                        bytearray.end() - 7,                   // length of </EOF!>
                        std::back_inserter( imagebytes ) );
            break;
          }
        }   
      }

      // Copy Image Bytes to a file on Disk
      std::cout << "Image bytes: " << imagebytes.size() << std::endl;
      std::ofstream os ( "/home/etsardou/copy_of_picture.png", std::ios::out | std::ofstream::binary );
      std::copy( imagebytes.begin(), imagebytes.end(), std::ostreambuf_iterator<byte>( os ) );
      os.close();        

      /* NOTE: 
       * I am assuming this is a blocking call here ?
       * If that is the case, then the method faceDetector::process, should be in a thread?
       */
      
      #define SETUP std::string, QrDetectionStrategies
      IRosServiceInvoker<SETUP>* t =  RosInvokerFactory::getInvoker<SETUP>( QR_DETECTION );
      #undef SETUP 
      /* NOTE:
       * I haven't changed this - We can use a RAMDISK for faster copy-write, 
       * or maybe pipe the data via Shared Memory Objects, or FIFO pipes?
       */
      t->setup( "/home/etsardou/copy_of_picture.png" );
      response = t->call_service();
      
      delete t;
      return response;
    } 
};

}
}
#endif
