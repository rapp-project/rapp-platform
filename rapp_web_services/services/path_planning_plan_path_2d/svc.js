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
 * [Path-planning-Plan-Path-2D] RAPP Platform web service.
 *
 *  @author Wojciech Dudek
 *  @mail wojciechsbox@gmail.com
 *  @copyright Rapp Project EU 2015
 */

var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_path_planning/planPath2d";


function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();
  rosMsg.user_name = req.username;
  rosMsg.map_name = req.body.map_name;
  rosMsg.robot_type = req.body.robot_type;
  rosMsg.algorithm = req.body.algorithm;
  rosMsg.start = req.body.start;
  rosMsg.goal = req.body.goal;

  /* ROS-Service response callback. */
  function callback(data){
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  /* ROS-Service onerror callback. */
  function onerror(e){
    resp.sendServerError();
  }

  /* Call ROS-Service. */
  ros.callService(rosSrvName, rosMsg,
    {success: callback, fail: onerror});
}



/***
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {String} response.error - Error message.
 *  @returns {Array} response.path - if plan_found is true, this is an array
 *    of waypoints from start to goal, where the first one equals start and
 *    the last one equals goal.
 *  @returns {Number} response.plan_found - Plan status:
 *    - 0 : path cannot be planned.
 *    - 1 : path found.
 *    - 2 : wrong map name.
 *    - 3 : wrong robot type.
 *    - 4 : wrong algorithm.
 *
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  error = rosbridge_msg.error_message;

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  response.path = rosbridge_msg.path;
  response.plan_found = rosbridge_msg.plan_found;
  return response;
}


module.exports = svcImpl;
