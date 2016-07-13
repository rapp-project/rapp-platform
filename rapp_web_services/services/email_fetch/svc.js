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
 * [Email-Fetch] RAPP Platform web service implementation.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 */



var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_email_receive/receive_email";


/**
 *  [Email-Fetch]
 *  Handles requests to email_fetch RAPP Platform Service
 *
 *  Service Implementation.
 *
 *
 */
function svcImpl ( req, resp, ros )
{
  if( ! req.body.email ){
    var response = new interfaces.client_res();
    response.error = 'Empty \"email\" argument';
    sendResponse( hop.HTTPResponseJson(response) );
    return;
  }
  if( ! req.body.server ){
    var response = new interfaces.client_res();
    response.error = 'Empty \"server\" argument';
    sendResponse( hop.HTTPResponseJson(response) );
    return;
  }

  var rosMsg = new interfaces.ros_req();
  rosMsg.email = req.body.email;
  rosMsg.password = req.body.passwd;
  rosMsg.server = req.body.server;
  rosMsg.port = req.body.port;
  rosMsg.requestedEmailStatus = req.body.email_status;
  rosMsg.fromDate = req.body.from_date;
  rosMsg.toDate = req.body.to_date;
  rosMsg.numberOfEmails = req.body.num_emails;

  // ROS-Service response callback.
  function callback(data){
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  // ROS-Service onerror callback.
  function onerror(e){
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
 *  @returns {Array} response.emails - Array of email entries.
 *  @returns {String} response.error - Error message
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var success = rosbridge_msg.status;
  var emails = rosbridge_msg.emails;
  var logMsg = 'Returning to client';

  var response = new interfaces.client_res();

  if( success < 0 ){
    // TODO What shall be returned to client????!!!!
    response.error = "Failed to fetch emails";
    return response;
  }


  for ( var i in emails ){
    var emailEntry = new interfaces.email_entry();
    emailEntry.sender = emails[i].sender;
    for ( var j in emails[i].receivers){
      emailEntry.receivers.push(emails[i].receivers[j]);
    }

    for ( var j in emails[i].attachmentPaths ){
      var fPath = emails[i].attachmentPaths[j];
      var attachment = new interfaces.attachment();
      var f = Fs.readFileSync(fPath);
      if( f )
      {
        attachment.filename = f.basename;
        attachment.data = f.data.toString('base64');
        emailEntry.attachments.push(attachment);
      }
    }

    emailEntry.date = emails[i].dateTime;
    emailEntry.subject = emails[i].subject;

    var body = '';
    try{
      body = Fs.readTextFile(emails[i].bodyPath);
    }
    catch(e){
      console.log(e);
      body = '';
    }
    emailEntry.body = body;

    response.emails.push(emailEntry);
  }

  return response;
}


module.exports = svcImpl;
