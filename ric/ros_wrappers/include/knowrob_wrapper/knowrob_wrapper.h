#include <string>
#include <iostream>

#include <json_prolog/prolog.h>

class KnowrobWrapper
{
  private:
    json_prolog::Prolog pl;
  
  public:
  
    KnowrobWrapper();

    std::vector<std::string> subclassesOfQuery(std::string ontology_class);
    std::vector<std::string> superclassesOfQuery(std::string ontology_class);
    std::vector<std::string> instanceFromClassQuery(std::string ontology_class);
};
