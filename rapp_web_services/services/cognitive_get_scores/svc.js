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
 * [Cognitive-get-scores] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 */

var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ----------------------------------------------------------------------- */
var svcParams = ENV.SERVICES.cognitive_get_scores;
var rosSrvName = svcParams.ros_srv_name;
/* ----------------------------------------------------------------------- */


/**
 *  [Cognitive-Get-Scores]
 *  Handles requests to cognitive_get_scores RAPP Platform Service
 *
 *  Service Implementation.
 *
 *
 */
function svcImpl ( req, resp, ros )
{
  var response = new interfaces.client_res();
  var rosMsg = new interfaces.ros_req();

  rosMsg.username = req.username;
  rosMsg.upToTime = req.body.up_to_time;
  rosMsg.testType = req.body.test_type;


  /***
   * ROS-Service response callback.
   */
  function callback(data){
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  /***
   * ROS-Service onerror callback.
   */
  function onerror(e){
    resp.sendServerError();
  }

  ros.callService(rosSrvName, rosMsg,
    {success: callback, fail: onerror});


}



/***
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *  @returns {Object} response - Response Object.
 *
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var trace = rosbridge_msg.trace;
  var success = rosbridge_msg.success;
  var error = rosbridge_msg.error;

  var testCategories = rosbridge_msg.testCategories;
  var testScores = rosbridge_msg.testScores;

  var logMsg = 'Returning to client';

  var response = new interfaces.client_res();
  if( error ){
    response.error = error;
    return response;
  }

  response.test_classes = testCategories;
  response.scores = testScores;

  return response;
}


module.exports = svcImpl;
