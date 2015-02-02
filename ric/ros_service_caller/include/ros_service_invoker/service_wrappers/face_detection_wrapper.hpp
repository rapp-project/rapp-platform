#ifndef FACE_DETECTION_WRAPPER_HPP
#define FACE_DETECTION_WRAPPER_HPP

/**
 * The FaceDetection invoker file
 */

#include <ros_service_invoker/ros_service_base.hpp>

// Includes the service the invoker must call
#include <rapp_platform_ros_communications/FaceDetectionRosSrv.h>

// Define for easier development
#define FACED_STR_SETUP std::string,FaceDetectionStrategies

/**
 * @enum FaceDetectionStrategies
 * @details This enum includes the different strategies concerning the 
 * invoker's setup. In this case, Face detection can setup the service
 * invokation via an std::string in two ways: the string can contain the
 * images URL or a JSON containing the image's URL.
 */
enum FaceDetectionStrategies
{
  STRING_IMAGE_URL,
  STRING_JSON
};

/**
 * @class FaceDetectionWrapper
 * @brief The Face Detection invoker. Inherits publicly the abstract
 * IRosServiceInvoker.
 */
template <class T, class S>
class FaceDetectionWrapper : public IRosServiceInvoker<T, S>
{

  // Typedef of the already existent Face Detection ROS service, from the
  // ros_nodes/face_detection ROS node
  typedef rapp_platform_ros_communications::FaceDetectionRosSrv FdSrv; 

  private:
    FdSrv srv; // The ROS service object
    std::string face_detection_service_url; // The face detection service URL

  public:
    // Constructor setting up the face detection service URL
    FaceDetectionWrapper(){
      face_detection_service_url = "/ric/face_detection_service";
    }

    // The virtual function's implementation
    void setup(T s);
    // Two implementations with std::string, having different strategies
    void setup_url(std::string s);
    void setup_json(std::string s);
    
    /**
     * @brief The service caller function. This must be called AFTER the 
     * setup has been called.
     */
    std::string call_service()
    {
      std::string ret;
      // Check for the face detection service's existence
      if(!ros::service::exists(face_detection_service_url, true)){
        // Throw exception
        ROS_ERROR("Service %s does not exist", 
          face_detection_service_url.c_str());
      }

      ROS_ERROR("Invoker sending image :%s", srv.request.imageFilename.c_str());
      // Calls the ROS face detection service
      if(ros::service::call(face_detection_service_url, srv))
      {
        // Prepares the response
        int n = 0;
        n = srv.response.faces_up_left.size();
        if(n > 0)
        {
          // Prepare the JSON response
          ret += "{\"faces\":[";
          for(unsigned int i = 0 ; i < n ; i++)
          {
            ret += "{\"top_left_x\":\"";
            ret += TOSTR(srv.response.faces_up_left[i].point.x);
            ret += "\",\"top_left_y\":\"";
            ret += TOSTR(srv.response.faces_up_left[i].point.y);
            ret += "\",\"bottom_right_x\":\"";
            ret += TOSTR(srv.response.faces_down_right[i].point.x);
            ret += "\",\"bottom_right_y\":\"";
            ret += TOSTR(srv.response.faces_down_right[i].point.y);
            ret += "\"},";
          }
          ret.erase(ret.size() - 1);
          ret += "]}";
        }
        else
        {
          return "{\"faces\":[]}";
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
void FaceDetectionWrapper<FACED_STR_SETUP>::setup_url(std::string s)
{
  srv.request.imageFilename = s;
}

/**
 * @brief Template specialization for the JSON strategy
 * NOTE: Not implemented yet
 */
template<>
void FaceDetectionWrapper<FACED_STR_SETUP>::setup_json(std::string s)
{
  std::cout << "Called JSON\n";
}

/**
 * @brief Template specialization for the std::string type
 */
template <>
void FaceDetectionWrapper<FACED_STR_SETUP>::setup(std::string s)
{
  // Here the strategy is checked and the appropriate setup function is called.
  switch(get_strategy())
  {
    case STRING_IMAGE_URL:
      setup_url(s);
      break;
    case STRING_JSON:
      setup_json(s);
      break;
    default: ;
      // Throw exception
  }
}

#endif // FACE_DETECTION_WRAPPER_HPP
