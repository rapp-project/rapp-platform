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
 * [Register-User-From-Store] RAPP Platform web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */



var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = ENV.SERVICES.register_user_from_store;



function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();

  rosMsg.username = req.body.username;
  rosMsg.password = req.body.password;
  rosMsg.device_token = req.body.device_token;
  rosMsg.language = req.body.language;


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
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *  @returns {Object} response - Response Object.
 *
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var response = new interfaces.client_res();
  var error = rosbridge_msg.error;

  if( error ){
    response.error = error;
    return response;
  }

  response.suggested_username = rosbridge_msg.suggested_username;
  return response;
}


module.exports = svcImpl;
