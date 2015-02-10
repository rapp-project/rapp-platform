#ifndef QR_DETECTION_WRAPPER_HPP
#define QR_DETECTION_WRAPPER_HPP

/**
 * The QrDetection invoker file
 */

#include <ros_service_invoker/ros_service_base.hpp>

// Includes the service the invoker must call
#include <rapp_platform_ros_communications/QrDetectionRosSrv.h>

// Define for easier development
#define QR_STR_SETUP std::string,QrDetectionStrategies

/**
 * @enum QrDetectionStrategies
 * @details This enum includes the different strategies concerning the 
 * invoker's setup.
 */
enum class QrDetectionStrategies
{
  STRING_IMAGE_URL
};


/**
 * @class QrDetectionWrapper
 * @brief The QR Detection invoker. Inherits publicly the abstract
 * IRosServiceInvoker.
 */
template <class T, class S>
class QrDetectionWrapper : public IRosServiceInvoker<T, S>
{
  public:
    
    // Typedef of the already existent Qr Detection ROS service, from the
    // ros_nodes/qr_detection ROS node
    typedef rapp_platform_ros_communications::QrDetectionRosSrv Srv; 

  private:
    Srv srv; // The ROS service object
    std::string service_url; // The qr detection service URL

  public:
    // Constructor setting up the qr detection service URL
    QrDetectionWrapper(){
      // TODO: Must take this from rosparam
      service_url = "/ric/qr_detection_service";
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

      // Calls the ROS service
      if(ros::service::call(service_url, srv))
      {
        // Prepares the response
        int n = 0;
        n = srv.response.qr_centers.size();
        if(n > 0)
        {
          // Prepare the JSON response
          ret += "{\"qrs\":[";
          for(unsigned int i = 0 ; i < n ; i++)
          {
            ret += "{\"qr_center_x\":\"";
            ret += TOSTR(srv.response.qr_centers[i].point.x);
            ret += "\",\"qr_center_y\":\"";
            ret += TOSTR(srv.response.qr_centers[i].point.y);
            ret += "\",\"qr_message\":\"";
            ret += TOSTR(srv.response.qr_messages[i]);
            ret += "\"},";
          }
          ret.erase(ret.size() - 1);
          ret += "]}";
        }
        else
        {
          return "{\"qrs\":[]}";
        }
      }
      else
      {
        // Throw exception
      }
      return ret;
    }
};

/**
 * @brief Template specialization for the setup with the URL strategy
 */
template<>
void QrDetectionWrapper<QR_STR_SETUP>::setup_url(std::string s)
{
  srv.request.imageFilename = s;
}


/**
 * @brief Template specialization for the std::string type
 */
template <>
void QrDetectionWrapper<QR_STR_SETUP>::setup(std::string s)
{
  // Here the strategy is checked and the appropriate setup function is called.
  switch(get_strategy())
  {
    case QrDetectionStrategies::STRING_IMAGE_URL:
    default:
      setup_url(s);
      break;
      // Throw exception
  }
}

#endif // QR_DETECTION_WRAPPER_HPP
