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

#include <string>
#include <iostream>
#include <json_prolog/prolog.h>
#include <rapp_platform_ros_communications/DbWrapperSrv.h>
#include <rapp_platform_ros_communications/StringArrayMsg.h>
#include <rapp_platform_ros_communications/OntologySimpleQuerySrv.h>



class KnowrobWrapper
{
  private:
    ros::NodeHandle nh_;  
    json_prolog::Prolog pl;    
    ros::ServiceClient mysql_write_client;
    ros::ServiceClient mysql_fetch_client;
  
  public:
  
    KnowrobWrapper(ros::NodeHandle nh);

    std::vector<std::string> subclassesOfQuery(std::string ontology_class);
    std::vector<std::string> superclassesOfQuery(std::string ontology_class);
    //std::vector<std::string> createInstanceQuery(std::string caller_arguments);
    rapp_platform_ros_communications::OntologySimpleQuerySrv::Response createInstanceQuery(std::string caller_arguments);
    std::vector<std::string> dumpOntologyQuery(std::string path);
    std::vector<std::string> loadOntologyQuery(std::string path);  
    std::vector<std::string> userInstancesFromClassQuery(std::string ontology_class);
    std::vector<std::string> checkIfClassExists(std::string classValue);
    std::vector<std::string> assignAttributeValueQuery(std::string caller_arguments);
};
