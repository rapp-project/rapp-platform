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
 * [Face-Detection] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');
var Fs = require( path.join(ENV.PATHS.INCLUDE_DIR, 'common', 'fileUtils.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_face_detection/detect_faces";



/**
 *  [Face-Detection]
 *  Handles requests to face_detection RAPP Platform Service
 *
 *  Service Implementation.
 *
 *
 */
function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();

  if( ! req.files.file ){
    response.error = "No image file received";
    resp.sendJson(response);
    return;
  }

  // Add values to the rosmsg.
  rosMsg.imageFilename = req.files.file[0];
  rosMsg.fast = req.body.fast;


  /***
   * ROS-Service response callback.
   */
  function callback(data){
    Fs.rmFile(req.files.file[0]);
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  /***
   * ROS-Service onerror callback.
   */
  function onerror(e){
    Fs.rmFile(req.files.file[0]);
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
 *  @returns {Array} response.faces - An array of face-objects.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function parseRosbridgeMsg( rosbridge_msg )
{
  var faces_up_left = rosbridge_msg.faces_up_left;
  var faces_down_right = rosbridge_msg.faces_down_right;
  var error = rosbridge_msg.error;
  var numFaces = faces_up_left.length;

  var logMsg = 'Returning to client';

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  for (var ii = 0; ii < numFaces; ii++)
  {
    /***
     * @namespace face
     * @property up_left_point - Face bounding box, up-left-point
     */
    var face = {
      up_left_point: {x: 0, y:0},
      down_right_point: {x: 0, y: 0}
    };

    face.up_left_point.x = faces_up_left[ii].point.x;
    face.up_left_point.y = faces_up_left[ii].point.y;
    face.down_right_point.x = faces_down_right[ii].point.x;
    face.down_right_point.y = faces_down_right[ii].point.y;
    response.faces.push( face );
  }

  return response;
}


module.exports = svcImpl;
