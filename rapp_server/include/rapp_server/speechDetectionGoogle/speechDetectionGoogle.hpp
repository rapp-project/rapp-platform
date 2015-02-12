#ifndef _RAPP_SPEECH_DETECTOR_GOOGLE_
#define _RAPP_SPEECH_DETECTOR_GOOGLE_

#include <rapp_server/speechDetectionGoogle/Includes.hxx>

namespace rapp {
namespace cloud {

class speechDetectionGoogle: public serviceHandler
{
  private:
    // Typedef of the already existent ROS service, from the
    // respective ROS node
    typedef rapp_platform_ros_communications::SpeechToTextSrv Srv; 
    Srv srv; // The ROS service object
    std::string service_url; // The ROS service URL

    /**
     * @brief The service caller function
     * @param img_url [std::string] The image for which the face detection
     * algorithm will be deployed
     * @return std::string JSON string with the results
     */
    std::string invoke_ros_service(std::string audio_url)
    {
      srv.request.filename.data = audio_url;
      
      std::string ret;
      // Check for the ROS service's existence
      if(!ros::service::exists(service_url, true)){
        // Throw exception
        ROS_ERROR("Service %s does not exist", service_url.c_str());
      }

      ret += "{";

      // Calls the ROS service
      if(ros::service::call(service_url, srv))
      {
        // Prepares the response
        int n = 0;
        n = srv.response.words.size();
        // Prepare the JSON response
        ret += "\"words\":[";
        for(unsigned int i = 0 ; i < n ; i++)
        {
          ret += "\"";
          ret += srv.response.words[i].data;
          ret += "\",";
        }
        ret.erase(ret.size() - 1);
        ret += "],";

        ret += "\"confidence\":\"";
        ret += TOSTR(srv.response.confidence);
        ret += "\",";

        ret += "\"alternatives\":[";
        for(unsigned int i = 0 ; i < srv.response.alternatives.size() ; i++)
        {
          ret += "{ \"words\":[";
          for(unsigned int j = 0 ; 
            j < srv.response.alternatives[i].s.size() ; j++)
          {
            ret += std::string("\"") + srv.response.alternatives[i].s[j].data +
              std::string("\",");
          }
          ret.erase(ret.size() - 1);
          ret += "]},";
        }
        ret.erase(ret.size() - 1);
        ret += "]";
      }
      else
      {
        // Throw exception
      }

      ret += "}";
      return ret;

    }

  public:

    speechDetectionGoogle(void)
    {
      service_url = "/ric/speech_to_text_service";
    }
    
    typedef char byte;
      
    std::string process ( const std::vector<byte> & bytearray )
    {
      std::vector<byte> filebytes;
      std::string response;

      // Search for the `<IMG!>` delimiter - then copy from that position, 
      // and up to the position of <EOF!>
      for ( unsigned int i = 0; i < bytearray.size(); i++ )
      {
        if ( (i + 5) < ( bytearray.size()-7 ) )
        {
          // NOTE: In order to avoid copying 5 bytes into string, maybe test  
          // first char is `<`?
          // NOTE: Check for <AUDIO> here!!
          if ( std::string( &bytearray[i], 5 ) == "<AUDIO>" )    // Find <IMG>
          {
            std::copy ( bytearray.begin() + i + 5,          // length of <IMG>
                        bytearray.end() - 7,                // length of </EOF!>
                        std::back_inserter( filebytes ) );
            break;
          }
        }   
      }

      // Copy audio Bytes to a file on Disk
      std::cout << "Audio bytes: " << filebytes.size() << std::endl;
      
      std::ofstream os ( "/home/etsardou/copy_of_audio.flac", 
        std::ios::out | std::ofstream::binary );
      
      std::copy( filebytes.begin(), filebytes.end(), 
        std::ostreambuf_iterator<byte>( os ) );
      
      os.close();        

      /* NOTE: 
       * I am assuming this is a blocking call here ?
       * If that is the case, then the method faceDetector::process, should be 
       * in a thread?
       * Manos: yes, thats blocking. A non blocking version exists that uses
       * a callback. The thread solution can work too.
       */
      return invoke_ros_service("/home/etsardou/copy_of_audio.flac") ;
    } 
};

}
}
#endif
