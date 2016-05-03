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
 * [Ontology-is-subsuperclass-of] RAPP Platform web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = ENV.SERVICES.ontology_is_subsuperclass_of.ros_srv_name;



/**
 *  [Ontology-is-subsuperclass-of] RAPP Platform front-end web service.
 *  Handles requests for ontology-is-subsuperclass-of query.
 *
 *  @function ontology_is_subsuperclass_of
 *
 *  @param {Object} args - Service input arguments (literal).
 *  @param {String} args.parent_class - The parent class name.
 *  @param {String} args.child_class - The child class name.
 *  @param {String} args.recursive - Recursive query.
 *
 *
 *  @returns {Object} response - JSON HTTPResponse Object.
 *    Asynchronous HTTP Response.
 *  @returns {Boolean} response.result - Query result.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 *
 */
function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();

  rosMsg.parent_class = req.body.parent_class;
  rosMsg.child_class = req.body.child_class;
  rosMsg.recursive = req.body.recursive;

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

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg,
    {success: callback, fail: onerror});

}



/***
 * Crafts response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {Boolean} response.result - Query result
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 *
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var result = rosbridge_msg.result;
  var success = rosbridge_msg.success;
  var error = rosbridge_msg.error;

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  response.result = result;

  return response;
}


module.exports = svcImpl;
