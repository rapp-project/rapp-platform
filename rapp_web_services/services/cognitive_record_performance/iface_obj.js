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


var clientRes = function( records, error ){
  records = records || {};
  error = error || '';
  var obj = {
    records: records,
    error: error
  };
  return obj;
};


var clientReq = function( user, testInstance, score ){
  user = user || '';
  testInstance = testInstance || '';
  score = score || 0;
  var obj = {
    user: user,
    test_instance: testInstance,
    score: score
  };
  return obj;
};


var rosReq = function( user, testInstance, score){
  user = user || '';
  testInstance = testInstance || '';
  score = parseInt(score) || 0;
  var obj = {
    username: user,
    test: testInstance,
    score: score
  };
  return obj;
};


var rosRes = function(){

};

exports.client_res = clientRes;
exports.client_req = clientReq;
exports.ros_req = rosReq;
exports.ros_res = rosRes;
