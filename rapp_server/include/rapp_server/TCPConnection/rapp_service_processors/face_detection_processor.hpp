#ifndef _FACE_DETECTION_PROCESSOR_
#define _FACE_DETECTION_PROCESSOR_

#include <rapp_server/TCPConnection/rapp_service_processors/processor_base.hpp>

class FaceDetectionProcessor: public BaseRappConnectionProcessor
{
  public:
    
    void process(const std::vector<char>& bytearray_, std::string& response)
    {
      std::vector<char> imagebytes;

      // Search for the `<IMG!>` delimiter - then copy from that position, 
      // and up to the position of <EOF!>
      for ( unsigned int i = 0; i < bytearray_.size(); i++ )
      {
        if ( (i + 5) < (bytearray_.size()-7) )
        {
          // NOTE: In order to avoid copying 5 bytes into string, maybe test 
          // first char is `<`
          if ( std::string( &bytearray_[i], 5 ) == "<DAT>" )
          {
            std::copy( bytearray_.begin() + i + 5,
              bytearray_.end() - 7,
              std::back_inserter( imagebytes) );
            break; // We got what we wanted, break out of the loop
          }
        }   
      }

      // Copy Image Bytes to a file on Disk
      std::cout << "Image bytes: " << imagebytes.size() << std::endl;
      std::ofstream os ( "/home/etsardou/copy_of_picture.png", 
        std::ios::out | std::ofstream::binary );
      std::copy( imagebytes.begin(), imagebytes.end(), 
        std::ostreambuf_iterator<char>( os ) );
      os.close();        
      // Always reply with an </EOF!> because the C++ client side expects this 
      // delimiter
      //reply_ = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 
      //8\r\nConnection: close\r\n\r\nBye!\r\n";

      #define SETUP std::string, FaceDetectionStrategies
      IRosServiceInvoker<SETUP>* t = 
        RosInvokerFactory::getInvoker<SETUP>(FACE_DETECTION);
      t->set_strategy(STRING_IMAGE_URL);
      t->setup("/home/etsardou/copy_of_picture.png");
      response = t->call_service();
      delete t;
    } 
};

#endif
