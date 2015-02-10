#ifndef SPEECH_DETECTION_GOOGLE_WRAPPER_HPP
#define SPEECH_DETECTION_GOOGLE_WRAPPER_HPP

/**
 * The invoker file
 */

#include <ros_service_invoker/ros_service_base.hpp>

// Includes the service the invoker must call
#include <rapp_platform_ros_communications/SpeechToTextSrv.h>

// Define for easier development
#define SPEECH_GOOGLE_STR_SETUP std::string,SpeechDetectionGoogleStrategies

/**
 * @enum QrDetectionStrategies
 * @details This enum includes the different strategies concerning the 
 * invoker's setup.
 */
enum class SpeechDetectionGoogleStrategies
{
  STRING_AUDIO_URL
};


/**
 * @class SpeechDetectionGoogleWrapper
 * @brief The speech Detection invoker. Inherits publicly the abstract
 * IRosServiceInvoker.
 */
template <class T, class S>
class SpeechDetectionGoogleWrapper : public IRosServiceInvoker<T, S>
{
  public:
    
    // Typedef of the already existent ROS service
    typedef rapp_platform_ros_communications::SpeechToTextSrv Srv; 

  private:
    Srv srv; // The ROS service object
    std::string service_url; // The service URL

  public:
    // Constructor setting up the qr detection service URL
    SpeechDetectionGoogleWrapper(){
      // TODO: Must take this from rosparam
      service_url = "/ric/speech_to_text_service";
    }

    // The virtual function's implementation
    void setup(T s){}
    // One implementation with std::string, having different strategies
    void setup_url(std::string s);
    
    /**
     * @brief The service caller function. This must be called AFTER the 
     * setup has been called.
     */
    std::string call_service()
    {
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
};

/**
 * @brief Template specialization for the setup with the URL strategy
 */
template<>
void SpeechDetectionGoogleWrapper<SPEECH_GOOGLE_STR_SETUP>::setup_url(std::string s)
{
  srv.request.filename.data = s;
}


/**
 * @brief Template specialization for the std::string type
 */
template <>
void SpeechDetectionGoogleWrapper<SPEECH_GOOGLE_STR_SETUP>::setup(std::string s)
{
  // Here the strategy is checked and the appropriate setup function is called.
  switch(get_strategy())
  {
    case SpeechDetectionGoogleStrategies::STRING_AUDIO_URL:
    default:
      setup_url(s);
      break;
      // Throw exception
  }
}

#endif // SPEECH_DETECTION_GOOGLE_WRAPPER_HPP
