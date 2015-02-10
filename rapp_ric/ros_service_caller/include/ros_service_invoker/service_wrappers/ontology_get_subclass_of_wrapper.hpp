#ifndef ONTOLOGY_GET_SUBCLASS_OF_WRAPPER_HPP
#define ONTOLOGY_GET_SUBCLASS_OF_WRAPPER_HPP

/**
 * The QrDetection invoker file
 */

#include <ros_service_invoker/ros_service_base.hpp>

// Includes the service the invoker must call
#include <rapp_platform_ros_communications/OntologySimpleQuerySrv.h>

// Define for easier development
#define STR_SETUP std::string,OntologySubclassOfStrategies


/**
 * @enum QrDetectionStrategies
 * @details This enum includes the different strategies concerning the 
 * invoker's setup.
 */
enum class OntologySubclassOfStrategies
{
  STRING_QUERY
};


/**
 * @class OntologySubclassOfWrapper
 * @brief The ontology subclass of invoker. Inherits publicly the abstract
 * IRosServiceInvoker.
 */
template <class T, class S>
class OntologySubclassOfWrapper : public IRosServiceInvoker<T, S>
{
  public:
    
    // Typedef of the already existent ontology subclass of ROS service, from the
    // corresponding ROS node
    typedef rapp_platform_ros_communications::OntologySimpleQuerySrv Srv; 

  private:
    Srv srv; // The ROS service object
    std::string service_url; // The qr detection service URL

  public:
    // Constructor setting up the qr detection service URL
    OntologySubclassOfWrapper(){
      // TODO: Must take this from rosparam
      service_url = "/ric/knowrob/subclasses_of";
    }

    // The virtual function's implementation
    void setup(T s){}
    // One implementation with std::string, having different strategies
    void setup_query(std::string s);
    
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
        n = srv.response.results.size();
        if(n > 0)
        {
          // Prepare the JSON response
          ret += "{\"subclasses\":[";
          for(unsigned int i = 0 ; i < n ; i++)
          {
            ret += "\"";
            ret += srv.response.results[i].data;
            ret += "\",";
          }
          ret.erase(ret.size() - 1);
          ret += "]}";
        }
        else
        {
          return "{\"subclasses\":[]}";
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
void OntologySubclassOfWrapper<STR_SETUP>::setup_query(std::string s)
{
  srv.request.query_term.data = s;
}


/**
 * @brief Template specialization for the std::string type
 */
template <>
void OntologySubclassOfWrapper<STR_SETUP>::setup(std::string s)
{
  // Here the strategy is checked and the appropriate setup function is called.
  switch(get_strategy())
  {
    case OntologySubclassOfStrategies::STRING_QUERY:
    default:
      setup_query(s);
      break;
      // Throw exception
  }
}


#undef STR_SETUP

#endif // QR_DETECTION_WRAPPER_HPP
