/******************************************************************************
Copyright 2015 RAPP

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

  Author: Athanassios Kintsakis
  contact: akintsakis@issel.ee.auth.gr

******************************************************************************/
#include <knowrob_wrapper/knowrob_wrapper.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>
#include <ros/package.h>
#include <fstream>

/**  
* @brief Default constructor 
*/ 
KnowrobWrapper::KnowrobWrapper(ros::NodeHandle nh):nh_(nh)
{
  mysql_write_client = nh_.serviceClient<rapp_platform_ros_communications::writeDataSrv>("/rapp/rapp_mysql_wrapper/tbl_user_write_data");
  mysql_fetch_client = nh_.serviceClient<rapp_platform_ros_communications::fetchDataSrv>("/rapp/rapp_mysql_wrapper/tbl_user_fetch_data");
  mysql_update_client = nh_.serviceClient<rapp_platform_ros_communications::updateDataSrv>("/rapp/rapp_mysql_wrapper/tbl_user_update_data");
}

/** 
* @brief Converts integer to string 
* @param a [int] The input integer 
* @return out [string] The output string 
*/ 
std::string intToString (int a)
{
    std::ostringstream temp;
    temp<<a;
    return temp.str();
}

/** 
* @brief Keeps only the final folder or file from a path
* @param str [string&] The input path 
* @return out [string] The output filename 
*/ 
std::string SplitFilename (const std::string& str)
{
  size_t found;
  found=str.find_last_of("/\\");
  return str.substr(0,found);
}

/** 
* @brief Checks if path exists
* @param fileName [char*] The input path 
* @return out [bool] True if file exists 
*/
bool checkIfFileExists(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

/** 
* @brief Splits string by delimiter
* @param str [string] The input string 
* @param sep [string] The delimiter
* @return arr [std::vector<std::string>] A vector with the string parts as splitted by the delimiter 
*/
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

/** 
* @brief Returns the ontology alias of the user
* @param user_id [string] The username of the user 
* @return ontology_alias [string] The ontology_alias of the user or possible error
*/
std::string KnowrobWrapper::get_ontology_alias(std::string user_id)
{
  std::string ontology_alias;
  rapp_platform_ros_communications::fetchDataSrv srv;
  rapp_platform_ros_communications::StringArrayMsg ros_string_array;
  std::vector<std::string> req_cols;
  req_cols.push_back("ontology_alias");
  ros_string_array.s.push_back("username");
  ros_string_array.s.push_back(user_id);
  srv.request.req_cols=req_cols;
  srv.request.where_data.push_back(ros_string_array);
  mysql_fetch_client.call(srv);
  if(srv.response.success.data!=true)
  {
    ontology_alias=srv.response.trace[0];
    ontology_alias=std::string("FAIL")+ontology_alias;
    return ontology_alias;
  }
  else
  {
    if(srv.response.res_data.size()<1)
    {
      ontology_alias=std::string("FAIL: User not found, incorrect username?");
      return ontology_alias;
    }
    ontology_alias=srv.response.res_data[0].s[0];
    if(ontology_alias==std::string("None"))
    {
      ontology_alias=create_ontology_alias_for_new_user(user_id);
    }
  }
  return ontology_alias;
}

/** 
* @brief Creates a new ontology alias for a user
* @param user_id [string] The username of the user 
* @return ontology_alias [string] The ontology_alias of the user or possible error
*/
std::string KnowrobWrapper::create_ontology_alias_for_new_user(std::string user_id)
{
  std::string ontology_alias;
  std::vector<std::string> instance_name;
  std::string query = std::string("rdf_instance_from_class(knowrob:'Person") + std::string("',A)");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  char status = results.getStatus();
  if(status==0)
  {
    ontology_alias=std::string("User was uninitialized (had no ontology alias), and initilization failed on ontology level");
    return ontology_alias;
  }

  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ;
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    instance_name.push_back(bdg["A"]);
  }

  ontology_alias=instance_name[0];
  std::vector<std::string> splitted = split(ontology_alias,std::string("#"));
  ontology_alias=splitted[1];

  rapp_platform_ros_communications::updateDataSrv srv;
  srv.request.set_cols.push_back("ontology_alias='"+std::string(ontology_alias)+std::string("'"));
  rapp_platform_ros_communications::StringArrayMsg ros_string_array;
  ros_string_array.s.push_back(std::string("username"));
  ros_string_array.s.push_back(user_id);
  srv.request.where_data.push_back(ros_string_array);
  mysql_update_client.call(srv);
  if(srv.response.success.data!=true)
  {
    ontology_alias=srv.response.trace[0];
    query = std::string("rdf_retractall(knowrob:'") + ontology_alias + std::string("',rdf:type,knowrob:'Person")+std::string("')");
    results = pl.query(query.c_str());
    status = results.getStatus();
    if(status==0)
    {
      ontology_alias=ontology_alias+std::string(" DB operation failed.. removing instance from ontology also failed...");
    }
    else if(status==3)
    {
      ontology_alias=ontology_alias+std::string(" DB operation failed..Remove from ontology Success");
    }
    std::string("FAIL")+ontology_alias;
    return ontology_alias;
  }

  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request dmp;
  dmp.file_url=std::string("/currentOntologyVersion.owl");
  KnowrobWrapper::dumpOntologyQuery(dmp);
  return ontology_alias;
}

/** 
* @brief Implements the record_user_cognitive_tests_performance ROS service 
* @param req [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response KnowrobWrapper::record_user_cognitive_tests_performance(rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request req)
{
  rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response res;
  if(req.test==std::string("") || req.score<0 || req.score>100 || req.timestamp<1 || req.patient_ontology_alias==std::string("") || req.test==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, one or more arguments not provided or out of range. Test score is >=0 and <=100 and timestamp is positive integers");
    res.error=std::string("Error, one or more arguments not provided or out of range. Test score is >=0 and <=100 and timestamp is positive integers");
    return res;
  }
  std::string timestamp = intToString(req.timestamp);
  std::string score = intToString(req.score);

  std::string query = std::string("cognitiveTestPerformed(B,knowrob:'")+req.patient_ontology_alias+std::string("',knowrob:'")+req.test+std::string("','")+timestamp+std::string("','")+score+std::string("',knowrob:'Person',knowrob:'CognitiveTestPerformed')");
  query=std::string("cognitiveTestPerformed(B,knowrob:'")+req.patient_ontology_alias+std::string("',knowrob:'")+req.test+std::string("','")+timestamp+std::string("','")+score+std::string("',knowrob:'Person',knowrob:'CognitiveTestPerformed')");
  res.trace.push_back(query);
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("Test performance entry insertion into ontology FAILED, either invalid test or patient alias"));
    res.error=std::string("Test performance entry insertion into ontology FAILED, either invalid test or patient alias");
    return res;
  }
  else if(status==3)
  {
    res.success=true;
  }
  std::vector<std::string> query_ret_tests;
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ;
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    std::string temp_query_tests=bdg["B"];
    query_ret_tests.push_back(temp_query_tests);
  }
  for(unsigned int i = 0 ; i < query_ret_tests.size() ; i++)
  {
    res.cognitive_test_performance_entry=(query_ret_tests[i]);
  }
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request dmp;
  dmp.file_url=std::string("currentOntologyVersion.owl");
  KnowrobWrapper::dumpOntologyQuery(dmp);
  return res;
}

/** 
* @brief Implements the create_cognitve_tests ROS service 
* @param req [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response KnowrobWrapper::create_cognitve_tests(rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request req)
{
  rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response res;
  if(req.test_type==std::string("") || req.test_difficulty<1 || req.test_path==std::string("") || req.test_subtype==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, one or more arguments not provided or out of range. Test variation and test difficulty are positive integers >0");
    res.error=std::string("Error, one or more arguments not provided or out of range.  Test variation and test difficulty are positive integers >0");
    return res;
  }

  std::string path = ros::package::getPath("rapp_cognitive_exercise");
  std::string temp_check_path=path+req.test_path;
  const char * c = temp_check_path.c_str();
  if(!checkIfFileExists(c))
  {
    res.success=false;
    res.trace.push_back(std::string("Test file does not exist in provided file path"));
    res.trace.push_back(req.test_path);
    res.error=std::string("Test file does not exist in provided file path");
    return res;
  }
  std::string difficulty = intToString(req.test_difficulty);
  std::string query = std::string("createCognitiveTest(knowrob:'")+req.test_type+std::string("',B,'")+difficulty+std::string("','")+req.test_path+std::string("',knowrob:'")+req.test_subtype+std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("Test insertion into ontology FAILED, possible error is test type/subtype invalid"));
    res.error=std::string("Test insertion into ontology FAILED, possible error is test type/subtype invalid");
    return res;
  }
  else if(status==3)
  {
    res.success=true;
  }
  std::vector<std::string> query_ret_tests;
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ;
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    std::string temp_query_tests=bdg["B"];
    query_ret_tests.push_back(temp_query_tests);
  }

  for(unsigned int i = 0 ; i < query_ret_tests.size() ; i++)
  {
    res.test_name=(query_ret_tests[i]);
  }
  std::string tmp_test_name;
  tmp_test_name.assign(res.test_name.c_str());
  std::vector<std::string> test_created=split(tmp_test_name,std::string("#"));

  if(test_created.size()==2)
  {
    for(unsigned int i=0 ; i < req.supported_languages.size(); i++)
    {
      query=std::string("rdf_assert(knowrob:'")+test_created[1]+std::string("',knowrob:supportedLanguages,knowrob:'")+req.supported_languages[i]+std::string("')");
      results = pl.query(query.c_str());
    }
  }
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request dmp;
  dmp.file_url=std::string("currentOntologyVersion.owl");
  KnowrobWrapper::dumpOntologyQuery(dmp);
  return res;
}

/** 
* @brief Implements the cognitive_tests_of_type ROS service 
* @param req [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response KnowrobWrapper::cognitive_tests_of_type(rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request req)
{
  rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response res;
  if(req.test_type==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, test_type empty");
    res.error=std::string("Error, test_type empty");
    return res;
  }
  if(req.test_language==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, language empty");
    res.error=std::string("Error, language empty");
    return res;
  }
  std::string query = std::string("cognitiveTestsOfType(knowrob:'")+req.test_type+std::string("',B,Path,Dif,Sub,knowrob:'")+req.test_language+std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("No tests of given type exist"));
    res.error=std::string("No tests of given type exist");
    return res;
  }
  else if(status==3)
  {
    res.success=true;
  }
  std::vector<std::string> query_ret_tests;
  std::vector<std::string> query_ret_scores;
  std::vector<std::string> query_ret_difficulty;
  std::vector<std::string> query_ret_file_paths;
  std::vector<std::string> query_ret_subtypes;
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ;
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;

    std::string temp_query_tests=bdg["B"];
    query_ret_tests.push_back(temp_query_tests);

    std::string temp_query_file_paths=bdg["Path"];
    query_ret_file_paths.push_back(temp_query_file_paths);

    std::string temp_query_difficulty=bdg["Dif"];
    query_ret_difficulty.push_back(temp_query_difficulty);

    std::string temp_query_subtypes=bdg["Sub"];
    query_ret_subtypes.push_back(temp_query_subtypes);
  }
  for(unsigned int i = 0 ; i < query_ret_tests.size() ; i++)
  {
    res.tests.push_back(query_ret_tests[i]);
    res.difficulty.push_back(query_ret_difficulty[i]);
    res.file_paths.push_back(query_ret_file_paths[i]);
    res.subtype.push_back(query_ret_subtypes[i]);
  }
  return res;
}

/** 
* @brief Implements the user_performance_cognitve_tests ROS service 
* @param req [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response KnowrobWrapper::user_performance_cognitve_tests(rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request req)
{
  rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response res;
  if(req.ontology_alias==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, ontology alias empty");
    res.error=std::string("Error, ontology alias empty");
    return res;
  }
  if(req.test_type==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, test type empty");
    res.error=std::string("Error, test type empty");
    return res;
  }
  std::string query=std::string("userCognitiveTestPerformance(knowrob:'")+req.ontology_alias+std::string("',knowrob:'")+req.test_type+std::string("',B,Dif,Timestamp,SC,P,SubType)");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    //res.trace.push_back(std::string("Class: ")+req.ontology_class+std::string(" does not exist"));
    res.error=std::string("No performance records exist for the user or invalid user or invalid test type");
    return res;
  }
  else if(status==3)
  {
    res.success=true;
  }
  std::vector<std::string> query_ret_tests;
  std::vector<std::string> query_ret_scores;
  std::vector<std::string> query_ret_difficulty;
  std::vector<std::string> query_ret_timestamps;
  std::vector<std::string> query_ret_subtypes;
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ;
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;

    std::string temp_query_tests=bdg["B"];
    query_ret_tests.push_back(temp_query_tests);
    
    std::string temp_query_difficulty=bdg["Dif"];
    query_ret_difficulty.push_back(temp_query_difficulty);
    
    std::string temp_query_timestamps=bdg["Timestamp"];
    query_ret_timestamps.push_back(temp_query_timestamps);
    
    std::string temp_query_scores=bdg["SC"];
    query_ret_scores.push_back(temp_query_scores);
    
    std::string tmp_query_subtypes=bdg["SubType"];
    query_ret_subtypes.push_back(tmp_query_subtypes);
  }
  for(unsigned int i = 0 ; i < query_ret_tests.size() ; i++)
  {
    res.tests.push_back(query_ret_tests[i]);
    res.scores.push_back(query_ret_scores[i]);
    res.difficulty.push_back(query_ret_difficulty[i]);
    res.timestamps.push_back(query_ret_timestamps[i]);
    res.subtypes.push_back(query_ret_subtypes[i]);
  }

  return res;
}

/** 
* @brief Implements the clear_user_cognitive_tests_performance_records ROS service 
* @param req [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response KnowrobWrapper::clear_user_cognitive_tests_performance_records(rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request req)
{
  rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response res;
  if(req.username==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, ontology alias empty");
    res.error=std::string("Error, ontology alias empty");
    return res;
  }
  std::string currentAlias=get_ontology_alias(req.username);
  if(req.test_type==std::string(""))
  {
    std::string query=std::string("rdf_has(P,knowrob:cognitiveTestPerformedPatient,knowrob:'")+currentAlias+std::string("'),rdf_retractall(P,L,S)");
    json_prolog::PrologQueryProxy results = pl.query(query.c_str());
    char status = results.getStatus();
    if(status==0)
    {
      res.success=false;
      res.error=std::string("No performance records exist for the user or invalid user or invalid test type");
      return res;
    }
    else if(status==3)
    {
      res.success=true;
    }
  }
  else
  {
    std::string query=std::string("rdf_has(A,rdf:type,knowrob:'")+req.test_type+std::string("'),rdf_has(P,knowrob:cognitiveTestPerformedTestName,A),rdf_has(P,knowrob:cognitiveTestPerformedPatient,knowrob:'")+currentAlias+std::string("'),rdf_retractall(P,L,S)");
    json_prolog::PrologQueryProxy results = pl.query(query.c_str());
    char status = results.getStatus();
    if(status==0)
    {
      res.success=false;
      res.error=std::string("No performance records exist for the user or invalid user or invalid test type");
      return res;
    }
    else if(status==3)
    {
      res.success=true;
    }
  }

  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request dmp;
  dmp.file_url=std::string("currentOntologyVersion.owl");
  KnowrobWrapper::dumpOntologyQuery(dmp);
  return res;
}

/** 
* @brief Implements the create_ontology_alias ROS service 
* @param req [rapp_platform_ros_communications::createOntologyAliasSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::createOntologyAliasSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::createOntologyAliasSrv::Response KnowrobWrapper::create_ontology_alias(rapp_platform_ros_communications::createOntologyAliasSrv::Request req)
{
  rapp_platform_ros_communications::createOntologyAliasSrv::Response res;
  if(req.username==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, empty username");
    res.error=std::string("Error, empty username");
    return res;
  }
  std::string currentAlias=get_ontology_alias(req.username);
  std::size_t found = currentAlias.find(std::string("FAIL"));
  if (found!=std::string::npos)
  {
    res.success=false;
    res.error=currentAlias;
    res.trace.push_back(currentAlias);
    return res;
  }
  res.ontology_alias=currentAlias;
  res.success=true;

  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request dmp;
  dmp.file_url=std::string("currentOntologyVersion.owl");
  KnowrobWrapper::dumpOntologyQuery(dmp);
  return res;
}

/** 
* @brief Implements the subclassesOf ROS service 
* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response KnowrobWrapper::subclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req)
{
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response res;
  if(req.ontology_class==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, empty ontology class");
    res.error=std::string("Error, empty ontology class");
    return res;
  }
  std::string query=std::string("");
  if(req.recursive==true)
  {
    query = std::string("superclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
  }
  else
  {
    query = std::string("direct_superclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
  }
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());

  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("Class: ")+req.ontology_class+std::string(" does not exist"));
    res.error=std::string("Class: ")+req.ontology_class+std::string(" does not exist");
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
    //start remove duplicates
    int i;
    int logic=0;
    std::string temp_query_result=bdg["A"];
    std::size_t found = temp_query_result.find(std::string("file:///"));
    if (found==std::string::npos)
    {
      for(int i=0; i<query_ret.size();i++)
      {
        if(query_ret.at(i)==temp_query_result)
        {
          logic=1;
        }
      }
      if(logic==0)
      {
        query_ret.push_back(temp_query_result);
      }
      //end removing duplicates
    }
  }
  for(unsigned int i = 0 ; i < query_ret.size() ; i++)
  {
    res.results.push_back(query_ret[i]);
  }
  return res;
}

/** 
* @brief Implements the superclassesOf ROS service 
* @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response 
*/  
rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response  KnowrobWrapper::superclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req)
{
  rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response res;
  if(req.ontology_class==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, empty ontology class");
    return res;
  }
  std::string query=std::string("");
  if(req.recursive==true)
  {
    query = std::string("subclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
  }
  else
  {
    query = std::string("direct_subclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
  }
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("Class: ")+req.ontology_class+std::string(" does not exist"));
    res.error=std::string("Class: ")+req.ontology_class+std::string(" does not exist");
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
    //start remove duplicates
    int i;
    int logic=0;
    std::string temp_query_result=bdg["A"];
    std::size_t found = temp_query_result.find(std::string("file:///"));
    if (found==std::string::npos)
    {
      for(int i=0; i<query_ret.size();i++)
      {
        if(query_ret.at(i)==temp_query_result)
        {
          logic=1;
        }
      }
      if(logic==0)
      {
        query_ret.push_back(temp_query_result);
      }
      //end removing duplicates
    }
  }
  for(unsigned int i = 0 ; i < query_ret.size() ; i++)
  {
    res.results.push_back(query_ret[i]);
  }
  return res;
}

/** 
* @brief Implements the isSubSuperclass ROS service 
* @param req [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response&] The ROS service response 
*/  
rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response  KnowrobWrapper::isSubSuperclassOfQuery(rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request req)
{
  rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response res;
  if(req.parent_class==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, empty ontology class");
    res.error=std::string("Error, empty ontology class");
    return res;
  }
  if(req.child_class==std::string(""))
  {
    res.success=false;
    res.trace.push_back("Error, empty other class");
    res.error=std::string("Error, empty other class");
    return res;
  }
  std::string query=std::string("");
  if(req.recursive==true)
  {
    query = std::string("superclassesOf_withCheck(knowrob:'") + req.parent_class + std::string("',A)");
  }
  else
  {
    query = std::string("direct_superclassesOf_withCheck(knowrob:'") + req.parent_class + std::string("',A)");
  }
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("Class: ")+req.parent_class+std::string(" does not exist"));
    res.error=std::string("Class: ")+req.parent_class+std::string(" does not exist");
    return res;
  }
  else if(status==3)
  {
    res.success=true;
  }
  std::vector<std::string> query_ret;

  int logic=0;
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ;
    it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    std::string temp_query_result=bdg["A"];
    std::vector<std::string> seperator=split(temp_query_result,std::string("#"));
    if(seperator[1]==req.child_class)
    {
      logic=1;
      break;
    }
  }
  if(logic==0)
  {
    res.result=false;
  }
  else
  {
    res.result=true;
  }
  return res;
}

/** 
* @brief Implements the createInstance ROS service 
* @param req [rapp_platform_ros_communications::createInstanceSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::createInstanceSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::createInstanceSrv::Response KnowrobWrapper::createInstanceQuery(rapp_platform_ros_communications::createInstanceSrv::Request req)
{
  rapp_platform_ros_communications::createInstanceSrv::Response res;
  if(req.username==std::string(""))
  {
    res.success=false;
    res.trace.push_back(std::string("Error, empty username"));
    res.error=std::string("Error, empty username");
    return res;
  }
  if(req.ontology_class==std::string(""))
  {
    res.success=false;
    res.trace.push_back(std::string("Error, empty ontology class"));
    res.error=std::string("Error,empty ontology class");
    return res;
  }
  std::string ontology_alias=get_ontology_alias(req.username);
  //ontology_alias
  std::size_t found = ontology_alias.find(std::string("FAIL"));
  if (found!=std::string::npos)
  {
    res.success=false;
    res.error=ontology_alias;
    return res;
  }
  //res.trace.push_back(ontology_alias);
  ////if(req.attribute_name.size()!=req.attribute_value.size())
  ////{
    ////res.success=false;
    ////std_msgs::String temp_std_msgs_string;
    ////temp_std_msgs_string.data=std::string("Error, attribute name array is not equal to attribute value array");
    ////res.trace.push_back(temp_std_msgs_string);
    ////return res;
  ////}

  ////std::vector<std::string> args;
 //// args=split(caller_arguments,",");
  ////std::vector<std::string> instance_name;
  ////if(args.size()<2)
  ////{
    ////int Number = args.size();//number to convert int a string
    ////std::string tempResult;//string which will contain the result
    ////std::stringstream convert; // stringstream used for the conversion
    ////convert << Number;//add the value of Number to the characters in the stream
    ////tempResult = convert.str();
    ////ret.push_back("Error, invalid number of arguments.. minimum required 2. You supplied: "+tempResult);
    ////return ret;
  ////}
  std::vector<std::string> instance_name;
  std::string query = std::string("instanceFromClass_withCheck_andAssign(knowrob:'") +req.ontology_class + std::string("',A,knowrob:'")+ontology_alias+std::string("')");
  res.trace.push_back(query);
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("Class: ")+req.ontology_class+std::string(" does not exist probably.. or ontology_alias for user exists in the mysqlDatabase and not in the ontology"));
    res.error=std::string("Class: ")+req.ontology_class+std::string("  does not exist probably.. or ontology_alias for user exists in the mysqlDatabase and not in the ontology");
    return res;
  }
  res.success=true;
  for(json_prolog::PrologQueryProxy::iterator it = results.begin() ;  it != results.end() ; it++)
  {
    json_prolog::PrologBindings bdg = *it;
    instance_name.push_back(bdg["A"]);
  }
  if(instance_name.size()==1)
  {
    instance_name=split(instance_name[0],"#");
    if(instance_name.size()==2)
    {
      res.instance_name=(std::string("Created instance name is: ")+instance_name[1]);
    }
    else
    {
      res.success=false;
      res.trace.push_back(std::string("Fatal Error, instance not created.. split to # error"));
      res.error=std::string("Fatal Error, instance not created.. split to # error");
      return res;
    }
  }
  else
  {
    res.success=false;
    res.trace.push_back(std::string("Fatal Error, instance not created.., retrieval error"));
    res.error=std::string("Fatal Error, instance not created.. retrieval error");
    return res;
  }
  //rapp_platform_ros_communications::writeDataSrv srv;
  //std_msgs::String temp_std_msgs_string;

  //temp_std_msgs_string.data="user_id";
  //srv.request.req_cols.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="ontology_class";
  //srv.request.req_cols.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="ontology_instance";
  //srv.request.req_cols.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="file_url";
  //srv.request.req_cols.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="comments";
  //srv.request.req_cols.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="created_timestamp";
  //srv.request.req_cols.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="updated_timestamp";
  //srv.request.req_cols.push_back(temp_std_msgs_string);

  //rapp_platform_ros_communications::StringArrayMsg temp_std_msgs_string_array;
  //temp_std_msgs_string.data="'"+req.username.data+"'";
  //temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="'"+req.ontology_class.data+"'";
  //temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="'"+instance_name[1]+"'";
  //temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="'"+req.file_url.data+"'";
  //temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="'"+req.comments.data+"'";
  //temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="curdate()";
  //temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);
  //temp_std_msgs_string.data="curdate()";
  //temp_std_msgs_string_array.s.push_back(temp_std_msgs_string);

  //srv.request.req_data.push_back(temp_std_msgs_string_array);

  //mysql_write_client.call(srv);
  //res.success=true;
  //if(srv.response.success.data!=true)
  //{
    //res.success=false;
    //std_msgs::String s;
    //for(unsigned int i = 0 ; i < srv.response.trace.size() ; i++)
    //{
      //s = srv.response.trace[i];
      //res.trace.push_back(s);
    //}
    //s.data=std::string("DB operation failed:, attempting to remove instance from ontology");
    //res.trace.push_back(s);
    //query = std::string("rdf_retractall(knowrob:'") +
    //instance_name[1] + std::string("',rdf:type,knowrob:'")+req.ontology_class.data+std::string("')");
    //results = pl.query(query.c_str());
    //char status = results.getStatus();
    //if(status==0)
    //{
      //s.data=std::string("Error, removing instance from ontology failed...");
      //res.trace.push_back(s);
      //return res;
    //}
    //else if(status==3)
    //{
      //s.data=std::string("Remove Success");
      //res.trace.push_back(s);
    //}

  //}
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Request dmp;
  dmp.file_url=std::string("currentOntologyVersion.owl");
  KnowrobWrapper::dumpOntologyQuery(dmp);

  return res;
}

/** 
* @brief Implements the dumpOntology ROS service 
* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::ontologyLoadDumpSrv::Response KnowrobWrapper::dumpOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req)
{
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;

  std::string path = getenv("HOME");
  path=path+std::string("/rapp_platform_files/");

  if(req.file_url.empty())   // || req.file_url==std::string("")
  {
    res.success=false;
    res.trace.push_back(std::string("Empty file path"));
    res.trace.push_back(req.file_url);
    res.error=std::string("Empty file path");
    return res;
  }
  size_t pathDepth = std::count(req.file_url.begin(), req.file_url.end(), '/');
  std::string str2 ("/");
  std::size_t found = req.file_url.find(str2);
  if (found!=std::string::npos && pathDepth>1)
  {
    std::string folderFromPath=SplitFilename(req.file_url);
    const char * c = folderFromPath.c_str();
    if(!checkIfFileExists(c))
    {
      res.success=false;
      res.trace.push_back(std::string("Path does not exist, invalid folder?"));
      res.trace.push_back(req.file_url);
      res.error=std::string("Path does not exist, invalid folder?");
      return res;
    }
  }



  req.file_url=path+req.file_url;


  std::string query = std::string("rdf_save('") + req.file_url + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());


  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    //std_msgs::String temp_std_msgs_string;
    //temp_std_msgs_string.data=std::string("Ontology dump failed");
    res.trace.push_back(std::string("Ontology dump failed"));
    res.error=std::string("Ontology dump failed");
    return res;
  }
  else if(status==3)
  {
    res.success=true;
  }
  return res;


}

/** 
* @brief Implements the loadOntology ROS service 
* @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::ontologyLoadDumpSrv::Response KnowrobWrapper::loadOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req)
{
  rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;
  if(req.file_url.empty())
  {
    res.success=false;
    res.trace.push_back(std::string("Empty file path"));
    res.trace.push_back(req.file_url);
    res.error=std::string("Empty file path");
    return res;
  }

  std::string path = getenv("HOME");
  path=path+std::string("/rapp_platform_files/");
  req.file_url=path+req.file_url;
  const char * c = req.file_url.c_str();
  if(!checkIfFileExists(c))
  {
    res.success=false;
    res.trace.push_back(std::string("File does not exist in provided file path: '")
      + std::string(req.file_url) + std::string("'"));
    res.trace.push_back(req.file_url);
    res.error=std::string("File does not exist in provided file path: '"
      + std::string(req.file_url) + std::string("'"));
    return res;
  }
  std::string query = std::string("rdf_load('") + req.file_url + std::string("')");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=false;
    res.trace.push_back(std::string("Ontology load failed"));
    res.error=std::string("Ontology load failed");
    return res;
  }
  else if(status==3)
  {
    res.success=true;
  }
  return res;
}

/** 
* @brief Implements the returnUserInstancesOfClass ROS service 
* @param req [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response&] The ROS service response 
*/ 
rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response KnowrobWrapper::user_instances_of_class(rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request req)
{
  rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response res;
  if(req.username==std::string(""))
  {
    res.success=false;
    res.trace.push_back(std::string("Error, empty username"));
    res.error=std::string("Error, empty username");
    return res;
  }
  if(req.ontology_class==std::string(""))
  {
    res.success=false;
    res.trace.push_back(std::string("Error, empty ontology class"));
    res.error=std::string("Error, empty ontology class");
    return res;
  }
  std::string ontology_alias=get_ontology_alias(req.username);
  std::string query = std::string("rdf_has(knowrob:'") + ontology_alias + std::string("',knowrob:'belongsToUser',A)");
  json_prolog::PrologQueryProxy results = pl.query(query.c_str());
  char status = results.getStatus();
  if(status==0)
  {
    res.success=true;
    res.trace.push_back(std::string("User has no instances"));
    res.error=std::string("INFO: user has no instances");
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
    //start removing duplicates
    int i;
    int logic=0;
    std::string temp_query_result=bdg["A"];
    if(req.ontology_class!=std::string("*"))
    {
      std::size_t found = temp_query_result.find(std::string("#")+req.ontology_class+std::string("_"));
      if (found!=std::string::npos)
      {
         query_ret.push_back(temp_query_result);
      }
    }
    else
    {
      query_ret.push_back(temp_query_result);
    }
    //end removing duplicates
  }
  for(unsigned int i = 0 ; i < query_ret.size() ; i++)
  {
    res.results.push_back(query_ret[i]);
  }

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

  return res;
}

//bool checkIfFileExists(const char* fname)
//{
  //if( access( fname, F_OK ) != -1 ) {
      //return true;
  //}
  //return false;
//}


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

//createCognitiveTest service
//returnAllCognitiveTestsOfType service
//recorduserCognitveTestperformance service


/** 
* @brief Implements the assertRetractAttribute ROS service 
* @param req [rapp_platform_ros_communications::assertRetractAttributeSrv::Request&] The ROS service request 
* @return res [rapp_platform_ros_communications::assertRetractAttributeSrv::Response&] The ROS service response 
*/ 
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
