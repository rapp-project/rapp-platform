#include <string>
#include <iostream>

#include <json_prolog/prolog.h>

#include <rapp_platform_ros_communications/DbWrapperSrv.h>

#include <rapp_platform_ros_communications/StringArrayMsg.h>

class KnowrobWrapper
{
  private:
    json_prolog::Prolog pl;    
  
  public:
  
    KnowrobWrapper();

    std::vector<std::string> subclassesOfQuery(std::string ontology_class);
    std::vector<std::string> superclassesOfQuery(std::string ontology_class);
    std::vector<std::string> createInstanceQuery(std::string caller_arguments);
    std::vector<std::string> dumpOntologyQuery(std::string path);
    std::vector<std::string> loadOntologyQuery(std::string path);  
    std::vector<std::string> userInstancesFromClassQuery(std::string ontology_class);
    std::vector<std::string> checkIfClassExists(std::string classValue);
    std::vector<std::string> checkIfAttributeAllowed(std::string subjectClass, std::string predicate, std::string objectClass);
};
