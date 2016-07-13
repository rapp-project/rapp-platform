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
 * [Human-Detection] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');
var Fs = require( path.join(ENV.PATHS.INCLUDE_DIR, 'common', 'fileUtils.js') );

var interfaces = require( path.join(__dirname, 'ihuman_obj.js') );

var rosSrvName = "/rapp/rapp_human_detection/detect_humans";



/**
 *  [Human-Detection]
 *  Handles requests to human_detection RAPP Platform Service
 *
 *  Service Implementation.
 *
 */
function svcImpl ( req, resp, ros )
{
  if( ! req.files.file ){
    var response = new interfaces.client_res();
    response.error = "No image file received";
    resp.sendJson(response);
    return;
  }

  var rosMsg = new interfaces.ros_req();
  rosMsg.imageFilename = req.files.file[0];

  // ROS-Service response callback.
  function callback(data){
    Fs.rmFile(req.files.file[0]);
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  // ROS-Service onerror callback.
  function onerror(e){
    Fs.rmFile(req.files.file[0]);
    var response = new interfaces.client_res();
    response.error = e;
    resp.sendJson(response);
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg, callback, onerror);
}


/***
 * Crafts response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {Array} response.humans - An array of face-objects.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var humans_up_left = rosbridge_msg.humans_up_left;
  var humans_down_right = rosbridge_msg.humans_down_right;
  var error = rosbridge_msg.error;
  var numHumans = humans_up_left.length;

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  for (var ii = 0; ii < numHumans; ii++)
  {
    /***
     * @namespace human
     * @property up_left_point - Face bounding box, up-left-point
     */
    var human = {
      up_left_point: {x: 0, y:0},
      down_right_point: {x: 0, y: 0}
    };

    human.up_left_point.x = humans_up_left[ii].point.x;
    human.up_left_point.y = humans_up_left[ii].point.y;
    human.down_right_point.x = humans_down_right[ii].point.x;
    human.down_right_point.y = humans_down_right[ii].point.y;
    response.humans.push( human );
  }

  return response;
}


module.exports = svcImpl;
