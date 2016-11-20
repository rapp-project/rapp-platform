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


/**
 * @fileOverview
 *
 * [Ontology-subclasses-of] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

const rosSrvName = "/rapp/rapp_mysql_wrapper/get_platform_user_info";



/***
 *  [Ontology-subclasses-of] RAPP Platform front-end web service.
 *  Handles requests for ontology-subclasses-of query.
 *
 * Service Implementation
 *
 */
function svcImpl(req, resp, ros) {
  var rosMsg = new interfaces.ros_req();
  rosMsg.username = req.username;

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
 *  @returns {Array} response.results - Query results.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 *
 */
function parseRosbridgeMsg(rosbridge_msg) {
  var response = new interfaces.client_res();

  if (rosbridge_msg.error) {
    response.error = error;
    return response;
  }

  response.name = rosbridge_msg.platform_user_info.name;
  response.surname = rosbridge_msg.platform_user_info.surname;
  response.language = rosbridge_msg.platform_user_info.language;

  for(let i in rosbridge_msg.platform_user_info.emails){
    var emailEntry = {
      user: rosbridge_msg.platform_user_info.emails[i].s[0],
      email_address: rosbridge_msg.platform_user_info.emails[i].s[1]
    };
    response.emails.push(emailEntry);
  }

  return response;
}


module.exports = svcImpl;
