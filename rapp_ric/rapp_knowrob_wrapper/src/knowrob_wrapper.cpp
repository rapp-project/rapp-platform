/**
MIT License (MIT)

Copyright (c) <2014> <Rapp Project EU>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Author: Athanassios Kintsakis
contact: akintsakis@issel.ee.auth.gr
**/

//subclassesOf  done tested
//superclassesOf  done tested
//loadontology  done tested
//dumpoontology done tested
//createinstance  done  tested
//getintancesOfUser
//attributesOfObject
//assignAttribute
//retractAttribute
//getSubjects
//getObjects




#include <knowrob_wrapper/knowrob_wrapper.h>

KnowrobWrapper::KnowrobWrapper(ros::NodeHandle nh):nh_(nh)
{
  mysql_write_client = nh_.serviceClient<rapp_platform_ros_communications::writeDataSrv>("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesWriteData");
  mysql_fetch_client = nh_.serviceClient<rapp_platform_ros_communications::fetchDataSrv>("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesFetchData");
  
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

//std::vector<std::string> KnowrobWrapper::checkIfClassExists(std::string classValue) //checked
//{
  //std::string query = std::string("owl_direct_subclass_of(knowrob:'") + 
  //classValue + std::string("',A)");
  //json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  //std::vector<std::string> report;

  //for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    //it != results.end() ; it++)
  //{
    //json_prolog::PrologBindings bdg = *it;
    //report.push_back(bdg["A"]);
  //}
  
  //if(report.size()==0)
  //{
    //report.clear();
    //report.push_back(std::string("Class entered: ") + classValue + std::string("  Either class does not exist or you used the Thing class which is not allowed"));
    //return report;
  //}
  //else
  //{
    //report.clear();
    //report.push_back(std::string("True"));
  //}
  //return report;
  
//}

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







rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response KnowrobWrapper::subclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req)
{
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response res;
  if(req.ontology_class.data==std::string(""))
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Error, empty ontology class");
    res.trace.push_back(temp_std_msgs_string);
    return res;
  } 
  std::string query = std::string("subclassesOf_withCheck(knowrob:'") + req.ontology_class.data + std::string("',A)"); 
  json_prolog::PrologQueryProxy results = pl.query(query.c_str()); 

  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Class: ")+req.ontology_class.data+std::string(" does not exist");
    res.trace.push_back(temp_std_msgs_string);
    return res;    
  }
  else if(status==3)
  {
    res.success=true;
  }
  std::vector<std::string> query_ret;  

  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    //std::string temp=bdg["A"];
    //res.results.push_back(temp_std_msgs_string);
    query_ret.push_back(bdg["A"]);
  }
  std_msgs::String temp_std_msgs_string;
  for(unsigned int i = 0 ; i < query_ret.size() ; i++)
  {
    temp_std_msgs_string.data = query_ret[i];
    res.results.push_back(temp_std_msgs_string);
  }
  return res;
}


rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response  KnowrobWrapper::superclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req)
{
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response res;
  std::string query = std::string("superclassesOf_withCheck(knowrob:'") + req.ontology_class.data + std::string("',A)");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Class: ")+req.ontology_class.data+std::string(" does not exist");
    res.trace.push_back(temp_std_msgs_string);
    return res;    
  }
  else if(status==3)
  {
    res.success=true;
  }
  std::vector<std::string> query_ret;  

  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    //std::string temp=bdg["A"];
    //res.results.push_back(temp_std_msgs_string);
    query_ret.push_back(bdg["A"]);
  }
  std_msgs::String temp_std_msgs_string;
  for(unsigned int i = 0 ; i < query_ret.size() ; i++)
  {
    temp_std_msgs_string.data = query_ret[i];
    res.results.push_back(temp_std_msgs_string);
  }
  return res;
}

//tested works
rapp_platform_ros_communications::createInstanceSrv::Response KnowrobWrapper::createInstanceQuery(rapp_platform_ros_communications::createInstanceSrv::Request req)
{
  rapp_platform_ros_communications::createInstanceSrv::Response res;  
  if(req.username.data==std::string(""))
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Error, empty username");
    res.trace.push_back(temp_std_msgs_string);
    return res;
  }  
  if(req.ontology_class.data==std::string(""))
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Error, empty ontology class");
    res.trace.push_back(temp_std_msgs_string);
    return res;
  }
  //if(req.attribute_name.size()!=req.attribute_value.size())
  //{
    //res.success=false;
    //std_msgs::String temp_std_msgs_string;
    //temp_std_msgs_string.data=std::string("Error, attribute name array is not equal to attribute value array");
    //res.trace.push_back(temp_std_msgs_string);
    //return res;    
  //}
  
  //std::vector<std::string> args;
 // args=split(caller_arguments,",");
  //std::vector<std::string> instance_name;
  //if(args.size()<2)
  //{
    //int Number = args.size();//number to convert int a string
    //std::string tempResult;//string which will contain the result
    //std::stringstream convert; // stringstream used for the conversion
    //convert << Number;//add the value of Number to the characters in the stream
    //tempResult = convert.str();
    //ret.push_back("Error, invalid number of arguments.. minimum required 2. You supplied: "+tempResult);
    //return ret;
  //}
  std::vector<std::string> instance_name;
  std::string query = std::string("instanceFromClass_withCheck(knowrob:'") + 
    req.ontology_class.data + std::string("',A)");  
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Class: ")+req.ontology_class.data+std::string(" does not exist");
    res.trace.push_back(temp_std_msgs_string);
    return res;    
  }

    
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;    
    instance_name.push_back(bdg["A"]);
  }
  
  if(instance_name.size()==1)
  {
    instance_name=split(instance_name[0],"#");
      if(instance_name.size()==2)
      {
        res.instance_name.data=(std::string("Created instance name is: ")+instance_name[1]);
      }
      else
      {
        res.success=false;
        std_msgs::String temp_std_msgs_string;
        temp_std_msgs_string.data=std::string("Fatal Error, instance name cannot be passed to DB, split to # error");      
        return res;
      }
  }
  else
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Fatal Error, instance name cannot be passed to DB, retrieval error");
    res.trace.push_back(temp_std_msgs_string);
    return res;  
  }

  rapp_platform_ros_communications::writeDataSrv srv;
  std_msgs::String temp_std_msgs_string;

  temp_std_msgs_string.data="user_id";
  srv.request.req_cols.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="ontology_class";
  srv.request.req_cols.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="ontology_instance";
  srv.request.req_cols.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="file_url";
  srv.request.req_cols.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="comments";
  srv.request.req_cols.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="created_timestamp";
  srv.request.req_cols.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="updated_timestamp";
  srv.request.req_cols.push_back(temp_std_msgs_string);  
  
  rapp_platform_ros_communications::StringArrayMsg temp_std_msgs_string_array;
  temp_std_msgs_string.data="'"+req.username.data+"'";  
  temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="'"+req.ontology_class.data+"'";  
  temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="'"+instance_name[1]+"'";  
  temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="'"+req.file_url.data+"'";  
  temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="'"+req.comments.data+"'"; 
  temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="curdate()";  
  temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  temp_std_msgs_string.data="curdate()";  
  temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
   
  srv.request.req_data.push_back(temp_std_msgs_string_array);
   
  mysql_write_client.call(srv);  
  res.success=true;
  if(srv.response.success.data!=true)
  {
    res.success=false;
    std_msgs::String s;
    for(unsigned int i = 0 ; i < srv.response.trace.size() ; i++)
    {      
      s = srv.response.trace[i];
      res.trace.push_back(s);
    }    
    s.data=std::string("DB operation failed:, attempting to remove instance from ontology");
    res.trace.push_back(s);
    query = std::string("rdf_retractall(knowrob:'") + 
    instance_name[1] + std::string("',rdf:type,knowrob:'")+req.ontology_class.data+std::string("')");
    results = pl.query(query.c_str());
    char status = results.getStatus();
    if(status==0)
    {
      s.data=std::string("Error, removing instance from ontology failed...");
      res.trace.push_back(s);
      return res;
    }
    else if(status==3)
    {
      s.data=std::string("Remove Success");
      res.trace.push_back(s);
    }
      
  }
  return res;  
}


//rapp_platform_ros_communications::assertRetractAttributeSrv::Response KnowrobWrapper::assertAttributeValue(rapp_platform_ros_communications::assertRetractAttributeSrv::Request req)
//{
  
  //rapp_platform_ros_communications::assertRetractAttributeSrv::Response res; 
  //if(res.attribute_names.size()!=res.attribute_values.size())
  //{
      //res.success=false;
      //s.data=std::string("Error, attribute_names and attribute_values sizes are not equal");
      //res.trace.push_back(s);
      //return res;
  //}
  ////ROS_WARN("Knowdfsdfsdlized");
  //std::string query = std::string("valueToAttribute_withCheck(knowrob:'") +   //instanceFromClass_withCheck
    //args[0] + std::string("',knowrob:'")+args[1]+std::string("',knowrob:'")+args[2]+std::string("')");
  //ret.push_back(query);
 //// return ret;
  //json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  
  

  ////std::vector<std::string> ret;
  //ret.clear();
  ////json_prolog::PrologQueryProxy thj;

  ////results.finish();
  ////json_prolog::PrologQueryProxy::iterator it;
  ////it.requestNextSolution();
  //char status = results.getStatus();
  //if(status==0)
  //{
    //ret.push_back(std::string("Assign Value Failed, you either supplied non existant instances, or wrong attribute for the instances"));
  //}
  //else if(status==3)
  //{
    //ret.push_back(std::string("Success"));
  //}  
  ////ROS_ERROR("Status %d", status);
  ////for(json_prolog::PrologQueryProxy::iterator it = results.begin() ; 
    ////it != results.end() ; it++)
  ////{
    //////json_prolog::PrologNextSolutionResponse resp; 
    //////json_prolog::PrologQueryProxy::iterator::requestNextSolution();
    //////it::requestNextSolution();
    ////json_prolog::PrologBindings bdg = *it;
    //////it.
    //////it.requestNextSolution();
    
    ////ret.push_back(bdg["A"]);
  ////}
  
  ////json_prolog::PrologQueryProxy::iterator it2 =results.end();
  //return ret;

  
  ////for(unsigned int i = 2 ; i < args.size() ; i=i+2)
  ////{
    //////rdf_assert(knowrob:'Person_qdaDeDZn',knowrob:worksAtFacility,knowrob:'HumanOccupationConstruct_pQmhNKHv')
    ////query = std::string("rdf_assert(knowrob:'") +     instanceName[1] + std::string("',knowrob:") +args[i] +  
    ////std::string(",knowrob:'") + args[i+1] +  std::string("'")  ;
    ////results = pl.query(query.c_str());
  ////}

  ////return ret;
//}

rapp_platform_ros_communications::ontologyLoadDumpSrv::Response KnowrobWrapper::dumpOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req)
{
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;
  
  std::string query = std::string("rdf_save('") + 
    req.file_url.data + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Ontology dump failed");
    res.trace.push_back(temp_std_msgs_string);
    return res;    
  }
  else if(status==3)
  {
    res.success=true; 
  }
  return res;


}

rapp_platform_ros_communications::ontologyLoadDumpSrv::Response KnowrobWrapper::loadOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req)
{
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;
  std::string query = std::string("rdf_load('") + 
    req.file_url.data + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    std_msgs::String temp_std_msgs_string;
    temp_std_msgs_string.data=std::string("Ontology dump failed");
    res.trace.push_back(temp_std_msgs_string);
    return res;    
  }
  else if(status==3)
  {
    res.success=true; 
  }
  return res;
}

//std::vector<std::string> KnowrobWrapper::userInstancesFromClassQuery(std::string ontology_class)
//{ 
  //rapp_platform_ros_communications::DbWrapperSrv srv;
  //std::vector<std::string> ret;  
  //std::string user_id;
  //std::string knowrobClass;
  //std::vector<std::string> args;
  
  //args=split(ontology_class,",");
  //user_id=args[0];
  //knowrobClass=args[1];  
  
  //std_msgs::String s;
  //s.data="ontology_instance";
  //srv.request.return_cols.push_back(s);  
  
  //std_msgs::String s1;
  //std_msgs::String s2;
  //s1.data="user_id";
  //s2.data=user_id;
  //rapp_platform_ros_communications::StringArrayMsg t1;
  //t1.s.push_back(s1);
  //t1.s.push_back(s2);  
  
  //std_msgs::String s3;
  //std_msgs::String s4;
  //s3.data="ontology_class";
  //s4.data="FoodOrDrink";
  //rapp_platform_ros_communications::StringArrayMsg t2;
  //t2.s.push_back(s3);
  //t2.s.push_back(s4);
  
  //srv.request.req_data.push_back(t1);
  //srv.request.req_data.push_back(t2);
  
  ////ros::NodeHandle n;
  ////ros::service::waitForService("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesFetchData", -1);
  ////ros::ServiceClient mysql_fetch_client = n.serviceClient<rapp_platform_ros_communications::DbWrapperSrv>("ric/db/mysql_wrapper_service/tblUsersOntologyInstancesFetchData");
 
  ////client.call(mysql_fetch_client);
  //for(unsigned int i = 0 ; i < srv.response.res_data.size() ; i++)
  //{
    //std::string sss;
    //sss = srv.response.res_data[i].s[0].data;
    //ret.push_back(sss);
  //}

  //return ret;
//}
