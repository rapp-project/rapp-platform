#include <knowrob_wrapper/knowrob_wrapper.h>


KnowrobWrapper::KnowrobWrapper()
{

}

std::vector<std::string> KnowrobWrapper::subclassOfQuery(std::string ontology_class)
{
  std::string query = std::string("owl_subclass_of(A, knowrob:'") + 
    ontology_class + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  std::vector<std::string> ret;

  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    ret.push_back(bdg["A"]);
  }
  return ret;
}
