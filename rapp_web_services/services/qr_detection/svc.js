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
 * [Qr-Detection] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */

var path = require('path');
var Fs = require( path.join(ENV.PATHS.INCLUDE_DIR, 'common', 'fileUtils.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_qr_detection/detect_qrs";



/**
 *  [Qr-Detection]
 *  Handles requests to qr_detection RAPP Platform Service
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
  ros.callService(rosSrvName, rosMsg, {success: callback, fail: onerror});
}


/***
 * Crafts response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {Array} response.qr_centers - An array of qr-center objects.
 *  @returns {Array} response.qr_messages - The array of the qr-messages. One
 *    qr_message string message for each found QR code.
 *  @returns {String} response.error - Error message
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var qrCenters = rosbridge_msg.qr_centers;
  var qrMessages = rosbridge_msg.qr_messages;
  var error = rosbridge_msg.error;

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  for (var ii = 0; ii < qrCenters.length; ii++)
  {
    var qrPoint = { x: 0, y: 0};

    qrPoint.x = qrCenters[ii].point.x;
    qrPoint.y = qrCenters[ii].point.y;
    response.qr_centers.push(qrPoint);
    response.qr_messages.push(qrMessages[ii]);
  }

  return response;
}


module.exports = svcImpl;
