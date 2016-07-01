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



/***
 * @fileOverview
 *
 * [Cognitive-get-history] RAPP Platform web service implementation.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 */


var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_cognitive_exercise/user_all_categories_history";



/**
 *  [Cognitive-Get-History]
 *  Handles requests to cognitive_get_history RAPP Platform Service
 *
 *  Service Implementation.
 */
function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();
  rosMsg.username = req.username;
  rosMsg.fromTime = req.body.from_time;
  rosMsg.toTime= req.body.to_time;
  rosMsg.testType= req.body.test_type;

  // ROS-Service response callback.
  function callback(data){
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  // ROS-Service onerror callback.
  function onerror(e){
    var response = new interfaces.client_res();
    response.error = e;
    resp.sendJson(response);
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg, {success: callback, fail: onerror});
}


/***
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {Objects} response.records - History records per test type.
 *    { ReasoningCts: [], AwarenessCts: [], ArithmeticCts: [] }
 *  @returns {String} response.error - Error message.
 *
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var trace = rosbridge_msg.trace;
  var success = rosbridge_msg.success;
  var error = rosbridge_msg.error;
  var recordsPerClass = rosbridge_msg.recordsPerTestType;
  var testClasses = rosbridge_msg.testCategories;

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  for( var ii = 0; ii < testClasses.length; ii++ ){
    try{
      response.records[testClasses[ii].toLowerCase()] =
        recordsPerClass[ii].records;
    }
    catch(e){
      response.records[testClasses[ii].toLowerCase()] = [];
    }
  }

  return response;
}

module.exports = svcImpl;
