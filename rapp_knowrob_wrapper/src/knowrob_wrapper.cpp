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
KnowrobWrapper::KnowrobWrapper(ros::NodeHandle nh) : nh_(nh) {
    mysql_register_user_ontology_alias_client = nh_.serviceClient<rapp_platform_ros_communications::registerUserOntologyAliasSrv>("/rapp/rapp_mysql_wrapper/register_user_ontology_alias");
    mysql_get_user_ontology_alias_client = nh_.serviceClient<rapp_platform_ros_communications::getUserOntologyAliasSrv>("/rapp/rapp_mysql_wrapper/get_user_ontology_alias");
    //mysql_update_client = nh_.serviceClient<rapp_platform_ros_communications::updateDataSrv>("/rapp/rapp_mysql_wrapper/tbl_user_update_data");
}

/**
 * @brief Dumps the ontology to default path
 */
void KnowrobWrapper::dump_ontology_now() {
    //rembember to remove path from here and put it to yaml
    std::string ontologyDefaultPath = std::string("currentOntologyVersion.owl");
    rapp_platform_ros_communications::ontologyLoadDumpSrv::Request dmp;
    dmp.file_url = ontologyDefaultPath;
    KnowrobWrapper::dumpOntologyQuery(dmp);
}

/**
 * @brief Check if a string is contained in a vector string
 * @param vec [vector<string>] The vector containing strings
 * @return a [string] The string to check if it is contained
 */
bool checkIfStringVectorContainsString(std::vector<std::string> vec, std::string a) {
    bool stringContained = false;
    for (int i = 0; i < vec.size(); i++) {
        if (vec.at(i) == a) {
            stringContained = true;
            break;
        }
    }
    return stringContained;
}

/**
 * @brief Checks if the second string is contained within the first string
 * @param a [string] The first string
 * @return b [string] The second String
 */
bool checkIfStringContainsString(std::string a, std::string b) {
    std::size_t found = a.find(b);
    if (found == std::string::npos) {
        return false;
    }
    return true;
}

/**
 * @brief Converts integer to string
 * @param a [int] The input integer
 * @return out [string] The output string
 */
std::string intToString(int a) {
    std::ostringstream temp;
    temp << a;
    return temp.str();
}

/**
 * @brief Keeps only the final folder or file from a path
 * @param str [string&] The input path
 * @return out [string] The output filename
 */
std::string SplitFilename(const std::string& str) {
    size_t found;
    found = str.find_last_of("/\\");
    return str.substr(0, found);
}

/**
 * @brief Checks if path exists
 * @param fileName [char*] The input path
 * @return out [bool] True if file exists
 */
bool checkIfFileExists(const char *fileName) {
    std::ifstream infile(fileName);
    return infile.good();
}

/**
 * @brief Splits string by delimiter
 * @param str [string] The input string
 * @param sep [string] The delimiter
 * @return arr [vector<string>] A vector with the string parts as splitted by the delimiter
 */
std::vector<std::string> split(std::string str, std::string sep) {
    char* cstr = const_cast<char*> (str.c_str());
    char* current;
    std::vector<std::string> arr;
    current = strtok(cstr, sep.c_str());
    while (current != NULL) {
        arr.push_back(current);
        current = strtok(NULL, sep.c_str());
    }
    return arr;
}

/**
 * @brief Implements the create_ontology_alias ROS service
 * @param req [rapp_platform_ros_communications::createOntologyAliasSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::createOntologyAliasSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::createOntologyAliasSrv::Response KnowrobWrapper::create_ontology_alias(rapp_platform_ros_communications::createOntologyAliasSrv::Request req) {
    rapp_platform_ros_communications::createOntologyAliasSrv::Response res;
    try {
        if (req.username == std::string("")) {
            throw std::string("Error, empty username");
        }
         std::string currentAlias = get_ontology_alias(req.username);
        // std::size_t found = currentAlias.find(std::string("FAIL"));
        // if (found != std::string::npos) {
        //     throw currentAlias;
        // }
        res.ontology_alias = currentAlias;
        res.success = true;
        KnowrobWrapper::dump_ontology_now();
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Returns the ontology alias of the user
 * @param username [string] The username of the user
 * @return ontology_alias [string] The ontology_alias of the user or possible error
 * @exception AppError
 */
std::string KnowrobWrapper::get_ontology_alias(std::string username) {
        std::string ontology_alias;
        rapp_platform_ros_communications::getUserOntologyAliasSrv srv;
        srv.request.username = username;
        mysql_get_user_ontology_alias_client.call(srv);
        if (srv.response.success != true) {
            throw std::string(std::string("FAIL: User not found, incorrect username?"));
        } else {
            ontology_alias = srv.response.ontology_alias;
            if (ontology_alias == std::string("None")) {
                ontology_alias = create_ontology_alias_for_new_user(username);
            }
        }
        return ontology_alias;
}

/**
 * @brief Creates a new ontology alias for a user
 * @param username [string] The username of the user
 * @return ontology_alias [string] The ontology_alias of the user or possible error
 * @exception AppError
 */
std::string KnowrobWrapper::create_ontology_alias_for_new_user(std::string username) {
    std::string ontology_alias;
    std::vector<std::string> instance_name;
    std::string query = std::string("rdf_instance_from_class(knowrob:'Person") + std::string("',A)");
    json_prolog::PrologQueryProxy results = pl.query(query.c_str());
    char status = results.getStatus();
    if (status == 0) {
        throw std::string("User was uninitialized (had no ontology alias), and initilization failed on ontology level");
    }
    for (json_prolog::PrologQueryProxy::iterator it = results.begin();
            it != results.end(); it++) {
        json_prolog::PrologBindings bdg = *it;
        instance_name.push_back(bdg["A"]);
    }

    ontology_alias = instance_name[0];
    std::vector<std::string> splitted = split(ontology_alias, std::string("#"));
    ontology_alias = splitted[1];
    rapp_platform_ros_communications::registerUserOntologyAliasSrv srv;
    srv.request.username = username;
    srv.request.ontology_alias = std::string(ontology_alias);
    mysql_register_user_ontology_alias_client.call(srv);
    if (srv.response.success != true) {
        std::string error = srv.response.trace[0];
        query = std::string("rdf_retractall(knowrob:'") + ontology_alias + std::string("',rdf:type,knowrob:'Person") + std::string("')");
        results = pl.query(query.c_str());
        status = results.getStatus();
        if (status == 0) {
            error = error + std::string(" DB operation failed.. removing instance from ontology also failed...");
        } else if (status == 3) {
            error = error + std::string(" DB operation failed..Remove from ontology Success");
        }
        throw std::string(std::string("FAIL") + error);
    }
    KnowrobWrapper::dump_ontology_now();
    return ontology_alias;
}

/**
 * @brief Implements the record_user_cognitive_tests_performance ROS service
 * @param req [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response&] The ROS service response
 */
rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response KnowrobWrapper::record_user_cognitive_tests_performance(rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Request req) {
    rapp_platform_ros_communications::recordUserPerformanceCognitiveTestsSrv::Response res;
    if (req.test == std::string("") || req.score < 0 || req.score > 100 || req.timestamp < 1 || req.patient_ontology_alias == std::string("") || req.test == std::string("")) {
        res.success = false;
        res.trace.push_back("Error, one or more arguments not provided or out of range. Test score is >=0 and <=100 and timestamp is positive integers");
        res.error = std::string("Error, one or more arguments not provided or out of range. Test score is >=0 and <=100 and timestamp is positive integers");
        return res;
    }
    std::string timestamp = intToString(req.timestamp);
    std::string score = intToString(req.score);

    std::string query = std::string("cognitiveTestPerformed(B,knowrob:'") + req.patient_ontology_alias + std::string("',knowrob:'") + req.test + std::string("','") + timestamp + std::string("','") + score + std::string("',knowrob:'Person',knowrob:'CognitiveTestPerformed')");
    query = std::string("cognitiveTestPerformed(B,knowrob:'") + req.patient_ontology_alias + std::string("',knowrob:'") + req.test + std::string("','") + timestamp + std::string("','") + score + std::string("',knowrob:'Person',knowrob:'CognitiveTestPerformed')");
    res.trace.push_back(query);
    json_prolog::PrologQueryProxy results = pl.query(query.c_str());

    char status = results.getStatus();
    if (status == 0) {
        res.success = false;
        res.trace.push_back(std::string("Test performance entry insertion into ontology FAILED, either invalid test or patient alias"));
        res.error = std::string("Test performance entry insertion into ontology FAILED, either invalid test or patient alias");
        return res;
    } else if (status == 3) {
        res.success = true;
    }
    std::vector<std::string> query_ret_tests;
    for (json_prolog::PrologQueryProxy::iterator it = results.begin();
            it != results.end(); it++) {
        json_prolog::PrologBindings bdg = *it;
        std::string temp_query_tests = bdg["B"];
        query_ret_tests.push_back(temp_query_tests);
    }
    for (unsigned int i = 0; i < query_ret_tests.size(); i++) {
        res.cognitive_test_performance_entry = (query_ret_tests[i]);
    }
    KnowrobWrapper::dump_ontology_now();
    return res;
}

/**
 * @brief Implements the create_cognitve_tests ROS service
 * @param req [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response KnowrobWrapper::create_cognitve_tests(rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Request req) {
    rapp_platform_ros_communications::createCognitiveExerciseTestSrv::Response res;
    try {
        if (req.test_type == std::string("") || req.test_difficulty < 1 || req.test_path == std::string("") || req.test_subtype == std::string("")) {
            throw std::string("Error, one or more arguments not provided or out of range. Test variation and test difficulty are positive integers >0");
        }

        std::string path = ros::package::getPath("rapp_cognitive_exercise");
        std::string temp_check_path = path + req.test_path;
        const char * c = temp_check_path.c_str();
        if (!checkIfFileExists(c)) {
            throw std::string("Test file does not exist in provided file path");
        }
        std::string difficulty = intToString(req.test_difficulty);
        std::string query = std::string("createCognitiveTest(knowrob:'") + req.test_type + std::string("',B,'") + difficulty + std::string("','") + req.test_path + std::string("',knowrob:'") + req.test_subtype + std::string("')");
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string("Test insertion into ontology FAILED, possible error is test type/subtype invalid");
        } else if (status == 3) {
            res.success = true;
        }
        std::vector<std::string> query_ret_tests;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin();
                it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            std::string temp_query_tests = bdg["B"];
            query_ret_tests.push_back(temp_query_tests);
        }
        for (unsigned int i = 0; i < query_ret_tests.size(); i++) {
            res.test_name = (query_ret_tests[i]);
        }
        std::string tmp_test_name;
        tmp_test_name.assign(res.test_name.c_str());
        std::vector<std::string> test_created = split(tmp_test_name, std::string("#"));
        if (test_created.size() == 2) {
            for (unsigned int i = 0; i < req.supported_languages.size(); i++) {
                query = std::string("rdf_assert(knowrob:'") + test_created[1] + std::string("',knowrob:supportedLanguages,knowrob:'") + req.supported_languages[i] + std::string("')");
                results = pl.query(query.c_str());
            }
        }
        KnowrobWrapper::dump_ontology_now();
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the cognitive_tests_of_type ROS service
 * @param req [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response KnowrobWrapper::cognitive_tests_of_type(rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Request req) {
    rapp_platform_ros_communications::cognitiveTestsOfTypeSrv::Response res;
    try {
        if (req.test_type == std::string("")) {
            throw std::string("Error, test_type empty");
        }
        if (req.test_language == std::string("")) {
            throw std::string("Error, language empty");
        }
        std::string query = std::string("cognitiveTestsOfType(knowrob:'") + req.test_type + std::string("',B,Path,Dif,Sub,knowrob:'") + req.test_language + std::string("')");
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string("No tests of given type exist");
        }
        res.success = true;
        std::vector<std::string> query_ret_tests;
        std::vector<std::string> query_ret_scores;
        std::vector<std::string> query_ret_difficulty;
        std::vector<std::string> query_ret_file_paths;
        std::vector<std::string> query_ret_subtypes;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin();
                it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            std::string temp_query_tests = bdg["B"];
            query_ret_tests.push_back(temp_query_tests);
            std::string temp_query_file_paths = bdg["Path"];
            query_ret_file_paths.push_back(temp_query_file_paths);
            std::string temp_query_difficulty = bdg["Dif"];
            query_ret_difficulty.push_back(temp_query_difficulty);
            std::string temp_query_subtypes = bdg["Sub"];
            query_ret_subtypes.push_back(temp_query_subtypes);
        }
        for (unsigned int i = 0; i < query_ret_tests.size(); i++) {
            res.tests.push_back(query_ret_tests[i]);
            res.difficulty.push_back(query_ret_difficulty[i]);
            res.file_paths.push_back(query_ret_file_paths[i]);
            res.subtype.push_back(query_ret_subtypes[i]);
        }
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the user_performance_cognitve_tests ROS service
 * @param req [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response KnowrobWrapper::user_performance_cognitve_tests(rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Request req) {
    rapp_platform_ros_communications::userPerformanceCognitveTestsSrv::Response res;
    try {
        if (req.ontology_alias == std::string("")) {
            throw std::string("Error, ontology alias empty");
        }
        if (req.test_type == std::string("")) {
            throw std::string("Error, test type empty");
        }
        std::string query = std::string("userCognitiveTestPerformance(knowrob:'") + req.ontology_alias + std::string("',knowrob:'") + req.test_type + std::string("',B,Dif,Timestamp,SC,P,SubType)");
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string("No performance records exist for the user or invalid user or invalid test type");
        }
        res.success = true;
        std::vector<std::string> query_ret_tests;
        std::vector<std::string> query_ret_scores;
        std::vector<std::string> query_ret_difficulty;
        std::vector<std::string> query_ret_timestamps;
        std::vector<std::string> query_ret_subtypes;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin();
                it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            std::string temp_query_tests = bdg["B"];
            query_ret_tests.push_back(temp_query_tests);
            std::string temp_query_difficulty = bdg["Dif"];
            query_ret_difficulty.push_back(temp_query_difficulty);
            std::string temp_query_timestamps = bdg["Timestamp"];
            query_ret_timestamps.push_back(temp_query_timestamps);
            std::string temp_query_scores = bdg["SC"];
            query_ret_scores.push_back(temp_query_scores);
            std::string tmp_query_subtypes = bdg["SubType"];
            query_ret_subtypes.push_back(tmp_query_subtypes);
        }
        for (unsigned int i = 0; i < query_ret_tests.size(); i++) {
            res.tests.push_back(query_ret_tests[i]);
            res.scores.push_back(query_ret_scores[i]);
            res.difficulty.push_back(query_ret_difficulty[i]);
            res.timestamps.push_back(query_ret_timestamps[i]);
            res.subtypes.push_back(query_ret_subtypes[i]);
        }
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the clear_user_cognitive_tests_performance_records ROS service
 * @param req [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response KnowrobWrapper::clear_user_cognitive_tests_performance_records(rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Request req) {
    rapp_platform_ros_communications::clearUserPerformanceCognitveTestsSrv::Response res;
    try {
        if (req.username == std::string("")) {
            throw std::string("Error, ontology alias empty");
        }
        std::string currentAlias = get_ontology_alias(req.username);
        if (req.test_type == std::string("")) {
            std::string query = std::string("rdf_has(P,knowrob:cognitiveTestPerformedPatient,knowrob:'") + currentAlias + std::string("'),rdf_retractall(P,L,S)");
            json_prolog::PrologQueryProxy results = pl.query(query.c_str());
            char status = results.getStatus();
            if (status == 0) {
                throw std::string("No performance records exist for the user or invalid user or invalid test type");
            } else if (status == 3) {
                res.success = true;
            }
        } else {
            std::string query = std::string("rdf_has(A,rdf:type,knowrob:'") + req.test_type + std::string("'),rdf_has(P,knowrob:cognitiveTestPerformedTestName,A),rdf_has(P,knowrob:cognitiveTestPerformedPatient,knowrob:'") + currentAlias + std::string("'),rdf_retractall(P,L,S)");
            json_prolog::PrologQueryProxy results = pl.query(query.c_str());
            char status = results.getStatus();
            if (status == 0) {
                throw std::string("No performance records exist for the user or invalid user or invalid test type");
            } else if (status == 3) {
                res.success = true;
            }
        }
        KnowrobWrapper::dump_ontology_now();
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the subclassesOf ROS service
 * @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response KnowrobWrapper::subclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req) {
    rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response res;
    try {
        if (req.ontology_class == std::string("")) {
            throw std::string("Error, empty ontology class");
        }
        std::string query = std::string("");
        if (req.recursive == true) {
            query = std::string("superclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
        } else {
            query = std::string("direct_superclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
        }
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string(std::string("Class: ") + req.ontology_class + std::string(" does not exist"));
        }
        res.success = true;
        std::vector<std::string> query_ret;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin();
                it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            std::string temp_query_result = bdg["A"];
            if (!checkIfStringContainsString(temp_query_result, std::string("file:///"))) {
                if (!checkIfStringVectorContainsString(query_ret, temp_query_result)) {
                    query_ret.push_back(temp_query_result);
                }
            }
        }
        for (unsigned int i = 0; i < query_ret.size(); i++) {
            res.results.push_back(query_ret[i]);
        }
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the superclassesOf ROS service
 * @param req [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response KnowrobWrapper::superclassesOfQuery(rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Request req) {
    rapp_platform_ros_communications::ontologySubSuperClassesOfSrv::Response res;
    try {
        if (req.ontology_class == std::string("")) {
            throw std::string("Error, empty ontology class");
        }
        std::string query = std::string("");
        if (req.recursive == true) {
            query = std::string("subclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
        } else {
            query = std::string("direct_subclassesOf_withCheck(knowrob:'") + req.ontology_class + std::string("',A)");
        }
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string(std::string("Class: ") + req.ontology_class + std::string(" does not exist"));
        }
        res.success = true;
        std::vector<std::string> query_ret;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin();
                it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            std::string temp_query_result = bdg["A"];
            if (!checkIfStringContainsString(temp_query_result, std::string("file:///"))) {
                if (!checkIfStringVectorContainsString(query_ret, temp_query_result)) {
                    query_ret.push_back(temp_query_result);
                }
            }
        }
        for (unsigned int i = 0; i < query_ret.size(); i++) {
            res.results.push_back(query_ret[i]);
        }
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the isSubSuperclass ROS service
 * @param req [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response KnowrobWrapper::isSubSuperclassOfQuery(rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Request req) {
    rapp_platform_ros_communications::ontologyIsSubSuperClassOfSrv::Response res;
    try {
        if (req.parent_class == std::string("")) {
            throw std::string("Error, empty ontology class");
        }
        if (req.child_class == std::string("")) {
            throw std::string("Error, empty other class");
        }
        std::string query = std::string("");
        if (req.recursive == true) {
            query = std::string("superclassesOf_withCheck(knowrob:'") + req.parent_class + std::string("',A)");
        } else {
            query = std::string("direct_superclassesOf_withCheck(knowrob:'") + req.parent_class + std::string("',A)");
        }
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string(std::string("Class: ") + req.parent_class + std::string(" does not exist"));
        }
        res.success = true;
        std::vector<std::string> query_ret;
        int logic = 0;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin();
                it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            std::string temp_query_result = bdg["A"];
            std::vector<std::string> seperator = split(temp_query_result, std::string("#"));
            if (seperator[1] == req.child_class) {
                logic = 1;
                break;
            }
        }
        if (logic == 0) {
            res.result = false;
        } else {
            res.result = true;
        }
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the createInstance ROS service
 * @param req [rapp_platform_ros_communications::createInstanceSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::createInstanceSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::createInstanceSrv::Response KnowrobWrapper::createInstanceQuery(rapp_platform_ros_communications::createInstanceSrv::Request req) {
    rapp_platform_ros_communications::createInstanceSrv::Response res;
    try {
        if (req.username == std::string("")) {
            throw std::string("Error, empty username");
        }
        if (req.ontology_class == std::string("")) {
            throw std::string("Error, empty ontology class");
        }
        std::string ontology_alias = get_ontology_alias(req.username);
        std::vector<std::string> instance_name;
        std::string query = std::string("instanceFromClass_withCheck_andAssign(knowrob:'") + req.ontology_class + std::string("',A,knowrob:'") + ontology_alias + std::string("')");
        res.trace.push_back(query);
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string(std::string("Class: ") + req.ontology_class + std::string(" does not exist probably.. or ontology_alias for user exists in the mysqlDatabase and not in the ontology"));
        }
        res.success = true;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin(); it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            instance_name.push_back(bdg["A"]);
        }
        if (instance_name.size() == 1) {
            instance_name = split(instance_name[0], "#");
            if (instance_name.size() == 2) {
                res.instance_name = (std::string("Created instance name is: ") + instance_name[1]);
            } else {
                throw std::string("Fatal Error, instance not created.. split to # error");
            }
        } else {
            throw std::string("Fatal Error, instance not created.., retrieval error");
        }
        KnowrobWrapper::dump_ontology_now();
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the dumpOntology ROS service
 * @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::ontologyLoadDumpSrv::Response KnowrobWrapper::dumpOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req) {
    rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;
    try {
        std::string path = getenv("HOME");
        path = path + std::string("/rapp_platform_files/");
        if (req.file_url.empty()) { // || req.file_url==std::string("")
            throw std::string("Empty file path");
        }
        size_t pathDepth = std::count(req.file_url.begin(), req.file_url.end(), '/');
        std::string str2("/");
        std::size_t found = req.file_url.find(str2);
        if (found != std::string::npos && pathDepth > 1) {
            std::string folderFromPath = SplitFilename(req.file_url);
            const char * c = folderFromPath.c_str();
            if (!checkIfFileExists(c)) {
                throw std::string("Path does not exist, invalid folder?");
            }
        }
        req.file_url = path + req.file_url;
        std::string query = std::string("rdf_save('") + req.file_url + std::string("')");
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string("Ontology dump failed");
        }
        res.success = true;
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the loadOntology ROS service
 * @param req [rapp_platform_ros_communications::ontologyLoadDumpSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::ontologyLoadDumpSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::ontologyLoadDumpSrv::Response KnowrobWrapper::loadOntologyQuery(rapp_platform_ros_communications::ontologyLoadDumpSrv::Request req) {
    rapp_platform_ros_communications::ontologyLoadDumpSrv::Response res;
    try {
        if (req.file_url.empty()) {
            throw std::string("Empty file path");
        }
        std::string path = getenv("HOME");
        path = path + std::string("/rapp_platform_files/");
        req.file_url = path + req.file_url;
        const char * c = req.file_url.c_str();
        if (!checkIfFileExists(c)) {
            throw std::string(std::string("File does not exist in provided file path: '")
                    + std::string(req.file_url) + std::string("'"));
        }
        std::string query = std::string("rdf_load('") + req.file_url + std::string("')");
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string("Ontology load failed");
        }
        res.success = true;
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the returnUserInstancesOfClass ROS service
 * @param req [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response&] The ROS service response
 * @exception AppError
 */
rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response KnowrobWrapper::user_instances_of_class(rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Request req) {
    rapp_platform_ros_communications::returnUserInstancesOfClassSrv::Response res;
    try {
        if (req.username == std::string("")) {
            throw std::string("Error, empty username");
        }
        if (req.ontology_class == std::string("")) {
            throw std::string("Error, empty ontology class");
        }
        std::string ontology_alias = get_ontology_alias(req.username);
        std::string query = std::string("rdf_has(knowrob:'") + ontology_alias + std::string("',knowrob:'belongsToUser',A)");
        json_prolog::PrologQueryProxy results = pl.query(query.c_str());
        char status = results.getStatus();
        if (status == 0) {
            throw std::string("User has no instances");
        }
        res.success = true;
        std::vector<std::string> query_ret;
        for (json_prolog::PrologQueryProxy::iterator it = results.begin();
                it != results.end(); it++) {
            json_prolog::PrologBindings bdg = *it;
            std::string temp_query_result = bdg["A"];
            if (!checkIfStringContainsString(temp_query_result, std::string("file:///"))) {
                if (!checkIfStringVectorContainsString(query_ret, temp_query_result)) {
                    query_ret.push_back(temp_query_result);
                }
            }
        }
        for (unsigned int i = 0; i < query_ret.size(); i++) {
            res.results.push_back(query_ret[i]);
        }
        return res;
    } catch (std::string error) {
        res.success = false;
        res.trace.push_back(error);
        res.error = error;
        return res;
    }
}

/**
 * @brief Implements the register_image_object_to_ontology ROS service
 * @param req [rapp_platform_ros_communications::registerImageObjectToOntologySrv::Request&] The ROS service request
 * @return res [rapp_platform_ros_communications::registerImageObjectToOntologySrv::Response&] The ROS service response
 */
rapp_platform_ros_communications::registerImageObjectToOntologySrv::Response KnowrobWrapper::register_image_object_to_ontology(rapp_platform_ros_communications::registerImageObjectToOntologySrv::Request req) {
    rapp_platform_ros_communications::registerImageObjectToOntologySrv::Response res;

    if (req.user_ontology_alias == std::string("") || req.timestamp < 1 || req.image_path == std::string("") || req.object_ontology_class == std::string("")) {
        res.success = false;
        res.trace.push_back("Error, one or more arguments not provided or out of range");
        res.error = std::string("Error, one or more arguments not provided or out of range");
        return res;
    }
    //std::string ontology_alias = get_ontology_alias(req.username);
    std::string timestamp = intToString(req.timestamp);
    std::string query = std::string("ObjectcreateObjectAndRegisterImage(Object,knowrob:'") + req.object_ontology_class + std::string("',knowrob:'") + req.user_ontology_alias + std::string("','") + timestamp + std::string("','") + req.image_path + std::string("')");
    //query = std::string("cognitiveTestPerformed(B,knowrob:'") + req.patient_ontology_alias + std::string("',knowrob:'") + req.test + std::string("','") + timestamp + std::string("','") + score + std::string("',knowrob:'Person',knowrob:'CognitiveTestPerformed')");
    json_prolog::PrologQueryProxy results = pl.query(query.c_str());
    char status = results.getStatus();
    if (status == 0) {
        res.success = false;
        res.trace.push_back(std::string("Test performance entry insertion into ontology FAILED, either invalid test or patient alias"));
        res.error = std::string("Test performance entry insertion into ontology FAILED, either invalid test or patient alias");
        return res;
    } else if (status == 3) {
        res.success = true;
    }
    std::vector<std::string> query_ret_tests;
    for (json_prolog::PrologQueryProxy::iterator it = results.begin();
            it != results.end(); it++) {
        json_prolog::PrologBindings bdg = *it;
        std::string temp_query_tests = bdg["Object"];
        query_ret_tests.push_back(temp_query_tests);
    }
    for (unsigned int i = 0; i < query_ret_tests.size(); i++) {
        res.object_entry = (query_ret_tests[i]);
    }
    KnowrobWrapper::dump_ontology_now();
    return res;
}
