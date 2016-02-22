/***
 * Copyright 2015 RAPP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Konstantinos Panayiotou
 * Contact: klpanagi@gmail.com
 *
 */


var clientRes = function( lang, questions, possAns, corrAns,
  testInstance, testType, testSubtype, error ){
  lang = lang || '';
  questions = questions || [];
  possAns = possAns || [];
  corrAns = corrAns || [];
  testInstance = testInstance || '';
  testType = testType || '';
  testSubtype = testSubtype || '';
  error = error || '';

  var obj = {
    lang: lang, questions: questions, possib_ans: possAns,
    correct_ans: corrAns, test_instance: testInstance,
    test_type: testType, test_subtype: testSubtype, error: error
  };
  return obj;
};


var clientReq = function( user, testType ){
  user = user || '';
  testType = testType || '';
  var obj = {
    user: user,
    test_type: testType
  };
  return obj;
};


var rosReq = function( user, testType){
  user = user || '';
  testType = testType || '';
  var obj = {
    username: user,
    testType: testType
  };
  return obj;
};


var rosRes = function(){

};

exports.client_res = clientRes;
exports.client_req = clientReq;
exports.ros_req = rosReq;
exports.ros_res = rosRes;
