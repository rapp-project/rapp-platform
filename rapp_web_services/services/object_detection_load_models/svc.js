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
 * Authors: Konstantinos Panayiotou, Maciej Stefańczyk
 * Contact: klpanagi@gmail.com, m.stefanczyk@elka.pw.edu.pl
 *
 */


/**
 * @fileOverview
 *
 * [Object Recognition - Load Models] RAPP Platform front-end web service.
 *
 *  @author Maciej Stefańczyk
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');
var Fs = require(path.join(ENV.PATHS.INCLUDE_DIR, 'common', 'fileUtils.js'));

var interfaces = require(path.join(__dirname, 'iface_obj.js'));

const rosSrvName = "/rapp/rapp_object_detection/load_models";


/**
 *  [Object Recognition - Load Models]
 *  Handles requests to object_detection/find_objects RAPP Platform Service
 *
 *  Service Implementation.
 */
function svcImpl(req, resp, ros) {
  var rosMsg = new interfaces.ros_req();
  rosMsg.names = req.body.names;
  rosMsg.user = req.body.user

  // ROS-Service response callback.
  function callback(data) {
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg(data);
    resp.sendJson(response);
  }

  // ROS-Service onerror callback.
  function onerror(e) {
    var response = new interfaces.client_res();
    response.error = e;
    resp.sendJson(response);
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg, {success: callback, fail: onerror});
}


/***
 * Crafts response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {String} response.error - Error message.
 */
function parseRosbridgeMsg(rosbridge_msg) {
  const result = rosbridge_msg.result;

  var response = new interfaces.client_res();
  response.result = result;

  if (result != 0) {
    if (result == -1) response.error = 'No models loaded';
    if (result == -2) response.error = 'No image to analyze';
  }

  return response;
}


module.exports = svcImpl;
