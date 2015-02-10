#include <knowrob_wrapper/knowrob_wrapper.h>


KnowrobWrapper::KnowrobWrapper()
{

}

std::vector<std::string> split(std::string str, std::string sep){
    char* cstr=const_cast<char*>(str.c_str());
    char* current;
    std::vector<std::string> arr;
    current=strtok(cstr,sep.c_str());
    while(current != NULL){
        arr.push_back(current);
        current=strtok(NULL, sep.c_str());
    }
    return arr;
}

std::vector<std::string> KnowrobWrapper::checkIfClassExists(std::string classValue) //checked
{
  std::string query = std::string("owl_direct_subclass_of(knowrob:'") + 
  classValue + std::string("',A)");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  std::vector<std::string> report;

  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    report.push_back(bdg["A"]);
  }
  
  if(report.size()==0)
  {
    report.clear();
    report.push_back(std::string("Class entered: ") + classValue + std::string("  Either class does not exist or you used the Thing class which is not allowed"));
    return report;
  }
  else
  {
    report.clear();
    report.push_back(std::string("True"));
  }
  return report;
  
}

//check if instance exists


//std::vector<std::string> KnowrobWrapper::checkIfAttributeAllowed(std::string subject, std::string predicate, std::string object) //checked
//{
  //std::vector<std::string> report;
  //std::string check=std::string("not ok");
  ////check if subject exists
  ////check if object exists
  //std::vector<std::string> tempSplit;
  //tempSplit=split(subject,"_");
  //std::string subjectClass=tempSplit[0];
  //tempSplit=split(object,"_");
  //std::string objectClass=tempSplit[0];
  
  
  //std::string query = std::string("rdf_has(knowrob:'") + 
  //predicate + std::string("',rdfs:domain,A)");
  //json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  ////std::vector<std::string> ret;

  //for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    //it != results.end() ; it++)
  //{
    //json_prolog::PrologBindings bdg = *it;
    ////ret.push_back(bdg["A"]);
    //std::string temp1 = bdg["A"];
    //tempSplit=split(temp1,"#");
    //if(tempSplit.size()>1)
    //{
      //report.push_back(tempSplit[1]);
      //if(tempSplit[1]==subjectClass)
      //{
        //check=std::string("Ok");        
      //}        
    //}    
  //}  
  //if(report.size()==0)
  //{
    ////report=std::string("ERROR, attribute: ")+predicate+ std::string(" does not exist for subject (domains) : ") +subject;
    //report.push_back(std::string("ERROR, attribute: ")+predicate+ std::string(" does not take ANY subject (domains) "));
    //return report;
  //}  
  //else if(check!=std::string("Ok"))
  //{
    //report.insert(report.begin(),std::string("ERROR, attribute: ")+predicate+ std::string(" does not exist for subject (domains) : ") +subject+ std::string("  ...possible subjects are listed below..."));
    //return report;
  //}
  
  //report.clear();
  //check=std::string("not ok");
  
  //query = std::string("rdf_has(knowrob:'") + 
  //predicate + std::string("',rdfs:range,A)");
  //results = pl.query(query.c_str());

  ////std::vector<std::string> ret;

  //for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    //it != results.end() ; it++)
  //{
    //json_prolog::PrologBindings bdg = *it;
    ////ret.push_back(bdg["A"]);
    //std::string temp1 = bdg["A"];
    //tempSplit=split(temp1,"#");
    //if(tempSplit.size()>1)
    //{
      //report.push_back(tempSplit[1]);
      //if(tempSplit[1]==objectClass)
      //{
        //check=std::string("Ok");        
      //}        
    //}    
  //}
  //if(report.size()==0)
  //{
    ////report=std::string("ERROR, attribute: ")+predicate+ std::string(" does not exist for subject (domains) : ") +subject;
    //report.push_back(std::string("ERROR, attribute: ")+predicate+ std::string(" does not take ANY object (range)"));
    //return report;
  //}  
  //else if(check!=std::string("Ok"))
  //{
    //report.insert(report.begin(),std::string("ERROR, attribute: ")+predicate+ std::string(" does not exist for object (range) : ") +object+ std::string("  ...possible subjects are listed below..."));
    //return report;
  //}
  
  //report.clear();
  //report.push_back("True");
  //return report;
  
//}







std::vector<std::string> KnowrobWrapper::subclassesOfQuery(std::string ontology_class)
{
  std::string query = std::string("owl_subclass_of(A, knowrob:'") + 
    ontology_class + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  char status = results.getStatus();
  ROS_ERROR("Status %d", status);
  std::vector<std::string> ret;

  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    ret.push_back(bdg["A"]);
  }
  return ret;
}

std::vector<std::string> KnowrobWrapper::superclassesOfQuery(std::string ontology_class)
{
  std::string query = std::string("owl_subclass_of(knowrob:'") + 
    ontology_class + std::string("',A)");
  std::vector<std::string> args;
  //std::string query = std::string("owl_has(A,rdf:type,knowrob:'") + 
   // args[1] + std::string("')");
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

std::vector<std::string> KnowrobWrapper::createInstanceQuery(std::string ontology_class)
{
  std::string query = std::string("instanceFromClass_withCheck(knowrob:'") + 
    ontology_class + std::string("',A)");
  std::vector<std::string> args;
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  std::vector<std::string> ret;
  char status = results.getStatus();
  if(status==0)
  {
    ret.push_back(std::string("Class: ")+ontology_class+std::string(" does not exist"));
  }
  else if(status==3)
  {
    ret.push_back(std::string("Success"));
  }
    
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    
    ret.push_back(bdg["A"]);
  }
  return ret;
}
std::vector<std::string> KnowrobWrapper::assignAttributeValueQuery(std::string caller_arguments)
{
  
  std::vector<std::string> args;
  args=split(caller_arguments,",");
  std::vector<std::string> ret;
  if(args.size()<3)
  {
    int Number = args.size();//number to convert int a string
    std::string tempResult;//string which will contain the result
    std::stringstream convert; // stringstream used for the conversion
    convert << Number;//add the value of Number to the characters in the stream
    tempResult = convert.str();
    ret.push_back("Error, invalid number of arguments.. minimum required 3. You supplied: "+tempResult);
    return ret;
  }
  //ROS_WARN("Knowdfsdfsdlized");
  std::string query = std::string("valueToAttribute_withCheck(knowrob:'") +   //instanceFromClass_withCheck
    args[0] + std::string("',knowrob:'")+args[1]+std::string("',knowrob:'")+args[2]+std::string("')");
  ret.push_back(query);
 // return ret;
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  
  

  //std::vector<std::string> ret;
  ret.clear();
  //json_prolog::PrologQueryProxy thj;

  //results.finish();
  //json_prolog::PrologQueryProxy::iterator it;
  //it.requestNextSolution();
  char status = results.getStatus();
  if(status==0)
  {
    ret.push_back(std::string("Assign Value Failed, you either supplied non existant instances, or wrong attribute for the instances"));
  }
  else if(status==3)
  {
    ret.push_back(std::string("Success"));
  }  
  //ROS_ERROR("Status %d", status);
  //for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    //it != results.end() ; it++)
  //{
    ////json_prolog::PrologNextSolutionResponse resp; 
    ////json_prolog::PrologQueryProxy::iterator::requestNextSolution();
    ////it::requestNextSolution();
    //json_prolog::PrologBindings bdg = *it;
    ////it.
    ////it.requestNextSolution();
    
    //ret.push_back(bdg["A"]);
  //}
  
  //json_prolog::PrologQueryProxy::iterator it2 =results.end();
  return ret;

  
  //rapp_platform_ros_communications::DbWrapperSrv srv;
  //std_msgs::String s;

  //s.data="user_id";
  //srv.request.return_cols.push_back(s);
  //s.data="ontology_class";
  //srv.request.return_cols.push_back(s);
  //s.data="ontology_instance";
  //srv.request.return_cols.push_back(s);
  //s.data="file_url";
  //srv.request.return_cols.push_back(s);
  //s.data="comments";
  //srv.request.return_cols.push_back(s);
  //s.data="created_timestamp";
  //srv.request.return_cols.push_back(s);
  //s.data="updated_timestamp";
  //srv.request.return_cols.push_back(s);
  
  //std_msgs::String s1;
  //rapp_platform_ros_communications::StringArrayMsg t1;
  //s1.data="'"+args[0]+"'";  
  //t1.s.push_back(s1);
  //s1.data="'"+args[1]+"'";  
  //t1.s.push_back(s1);
  //s1.data="'"+instanceName[1]+"'";  
  //t1.s.push_back(s1);
  //s1.data="'url_something'";  
  //t1.s.push_back(s1);
  //s1.data="'comments_something'";  
  //t1.s.push_back(s1);
  //s1.data="curdate()";  
  //t1.s.push_back(s1);
  //s1.data="curdate()";  
  //t1.s.push_back(s1);
   
  //srv.request.req_data.push_back(t1);
  
  
  //ros::NodeHandle n;
  //ros::service::waitForService("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesWriteData", -1);
  //ros::ServiceClient client = n.serviceClient<rapp_platform_ros_communications::DbWrapperSrv>("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesWriteData");
 
  //client.call(srv);
  
  //ret.push_back("Write to DB report: "+srv.response.report.data);
  
  //for(unsigned int i = 2 ; i < args.size() ; i=i+2)
  //{
    ////rdf_assert(knowrob:'Person_qdaDeDZn',knowrob:worksAtFacility,knowrob:'HumanOccupationConstruct_pQmhNKHv')
    //query = std::string("rdf_assert(knowrob:'") +     instanceName[1] + std::string("',knowrob:") +args[i] +  
    //std::string(",knowrob:'") + args[i+1] +  std::string("'")  ;
    //results = pl.query(query.c_str());
  //}

  //return ret;
}

std::vector<std::string> KnowrobWrapper::dumpOntologyQuery(std::string path)
{
  std::string query = std::string("rdf_save('") + 
    path + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  std::vector<std::string> ret;

  //for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    //it != results.end() ; it++)
  //{
    //json_prolog::PrologBindings bdg = *it;
    //ret.push_back(bdg["A"]);
  //}
  return ret;
}

std::vector<std::string> KnowrobWrapper::loadOntologyQuery(std::string path)
{
  std::string query = std::string("rdf_load('") + 
    path + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  std::vector<std::string> ret;

  //for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    //it != results.end() ; it++)
  //{
    //json_prolog::PrologBindings bdg = *it;
    //ret.push_back(bdg["A"]);
  //}
  return ret;
}

std::vector<std::string> KnowrobWrapper::userInstancesFromClassQuery(std::string ontology_class)
{ 
  rapp_platform_ros_communications::DbWrapperSrv srv;
  std::vector<std::string> ret;  
  std::string user_id;
  std::string knowrobClass;
  std::vector<std::string> args;
  
  args=split(ontology_class,",");
  user_id=args[0];
  knowrobClass=args[1];  
  
  std_msgs::String s;
  s.data="ontology_instance";
  srv.request.return_cols.push_back(s);  
  
  std_msgs::String s1;
  std_msgs::String s2;
  s1.data="user_id";
  s2.data=user_id;
  rapp_platform_ros_communications::StringArrayMsg t1;
  t1.s.push_back(s1);
  t1.s.push_back(s2);  
  
  std_msgs::String s3;
  std_msgs::String s4;
  s3.data="ontology_class";
  s4.data="FoodOrDrink";
  rapp_platform_ros_communications::StringArrayMsg t2;
  t2.s.push_back(s3);
  t2.s.push_back(s4);
  
  srv.request.req_data.push_back(t1);
  srv.request.req_data.push_back(t2);
  
  ros::NodeHandle n;
  ros::service::waitForService("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesFetchData", -1);
  ros::ServiceClient client = n.serviceClient<rapp_platform_ros_communications::DbWrapperSrv>("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesFetchData");
 
  client.call(srv);
  for(unsigned int i = 0 ; i < srv.response.res_data.size() ; i++)
  {
    std::string sss;
    sss = srv.response.res_data[i].s[0].data;
    ret.push_back(sss);
  }

  return ret;
}
