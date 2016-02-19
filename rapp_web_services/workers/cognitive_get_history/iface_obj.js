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


var clientReq = function( user, timeFrom, timeTo, testType ){
  user = user || '';
  timeFrom = timeFrom || 0;
  timeTo = timeTo || 0;
  testType = testType || '';
  var obj = {
    user: user,
    from_time: timeFrom,
    to_time: timeTo,
    test_type: testType
  };
  return obj;
};


var rosReq = function( user, timeFrom, timeTo, testType){
  user = user || '';
  timeFrom = timeFrom || 0;
  timeTo = timeTo || 0;
  testType = testType || '';
  var obj = {
    username: user,
    fromTime: timeFrom,
    toTime: timeTo,
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
