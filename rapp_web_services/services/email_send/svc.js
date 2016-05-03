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
 * [Email-Send] RAPP Platform web service implementation.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 */


var path = require('path');
var zip = require( path.join(ENV.PATHS.INCLUDE_DIR, 'common', 'zip.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = ENV.SERVICES.email_send.ros_srv_name;



/**
 *  [Email-Send]
 *  Handles requests to email_send RAPP Platform Service
 *
 *  Service Implementation.
 *
 *
 */
function svcImpl ( req, resp, ros )
{
  var response = new interfaces.client_res();

  /* ------------------------------------------ */

  //if( ! req.body.email ){
    //error = 'Empty \"email\" argument';
    //response.error = error;
    //sendResponse( hop.HTTPResponseJson(response) );
    //return;
  //}
  //if( ! req.body.recipients ){
    //error = 'Empty \"recipients\" argument';
    //response.error = error;
    //sendResponse( hop.HTTPResponseJson(response) );
    //return;
  //}
  //if( ! req.body.server ){
    //error = 'Empty \"server\" argument';
    //response.error = error;
    //sendResponse( hop.HTTPResponseJson(response) );
    //return;
  //}

  var _files = [];


  if( zip.isZipFile(req.files[0]) ){
    _files = zip.unzip(req.files[0]).filepaths;
  }
  else{
    _files.push(req.files[0]);
  }

  var rosMsg = new interfaces.ros_req();
  rosMsg.userEmail = req.body.email;
  rosMsg.password = req.body.passwd;
  rosMsg.server = req.body.server;
  rosMsg.port = req.body.port;
  rosMsg.recipients = req.body.recipients;
  rosMsg.body = req.body.body;
  rosMsg.subject = req.body.subject;
  rosMsg.files = _files;


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
 *  @returns {Array} response.faces - An array of face-objects.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var success = rosbridge_msg.status;
  var logMsg = 'Returning to client';

  var response = new interfaces.client_res();

  if( success < 0 ){
    response.error = "Failed to send email";
  }

  return response;
}


module.exports = svcImpl;
