#include <string>
#include <iostream>

#include <json_prolog/prolog.h>

class KnowrobWrapper
{
  private:
    json_prolog::Prolog pl;
  
  public:
  
    KnowrobWrapper();

    std::vector<std::string> subclassOfQuery(std::string ontology_class);
};
