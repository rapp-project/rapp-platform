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
 * [Ontology-superclasses-of] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */

var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

const rosSrvName = "/rapp/rapp_knowrob_wrapper/superclasses_of";


/**
 *  [Ontology-superclasses-of] RAPP Platform front-end web service.
 *  Handles requests for ontology-superclasses-of query.
 *
 * Service Implementation
 *
 */
function svcImpl(req, resp, ros) {
  var rosMsg = new interfaces.ros_req();
  rosMsg.ontology_class = req.body.ontology_class;
  rosMsg.recursive = req.body.recursive;

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
 *  @returns {String} response.error - Error message
 *
 */
function parseRosbridgeMsg(rosbridge_msg) {
  const results = rosbridge_msg.results;
  const trace = rosbridge_msg.trace;
  const success = rosbridge_msg.success;
  const error = rosbridge_msg.error;

  var response = new interfaces.client_res();

  if (error) {
    response.error = error;
    return response;
  }

  for (let ii = 0; ii < results.length; ii++) {
    response.results.push(results[ii]);
  }

  return response;
}


module.exports = svcImpl;
