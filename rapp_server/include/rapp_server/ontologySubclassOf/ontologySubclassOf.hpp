#ifndef _RAPP_ONTOLOGY_SUBCLASS_OF_
#define _RAPP_ONTOLOGY_SUBCLASS_OF_

#include <rapp_server/ontologySubclassOf/Includes.hxx>

namespace rapp {
namespace cloud {

class ontologySubclassOf: public serviceHandler
{
  private:
    // Typedef of the already existent ROS service, from the
    // respective ROS node
    typedef rapp_platform_ros_communications::OntologySimpleQuerySrv Srv; 
    Srv srv; // The ROS service object
    std::string service_url; // The ROS service URL
    ros::NodeHandle nh_; // The ROS node handle

    /**
     * @brief The service caller function
     * @param ontology_class [std::string] The ontology class for which the 
     * subclasses of will be found
     * @return std::string JSON string with the results
     */
    std::string invoke_ros_service(std::string ontology_class)
    {
      srv.request.query_term.data = ontology_class;
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

  public:

    ontologySubclassOf(void)
    {
      if(!nh_.getParam("/ontology_subclass_of_topic", service_url))
      {
        ROS_ERROR("Ontology subclass of service parameter not found");
      }
    }
    
    typedef char byte;
      
    std::string process ( const std::vector<byte> & bytearray )
    {

      std::string ontology_class;
      //------Testing purposes--------//
      ontology_class = "Food";
      //------------------------------//

      // NOTE: Extract the input data from the bytearray. Proposal: Create
      // bytearray parser, returning its specific parts.
      return invoke_ros_service(ontology_class) ;
    } 
};

}
}
#endif
