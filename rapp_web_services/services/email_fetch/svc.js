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


var hop = require('hop');
var path = require('path');
var util = require('util');

var PKG_DIR = ENV.PATHS.PKG_DIR;
var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var svcUtils = require(path.join(INCLUDE_DIR, 'common',
    'svc_utils.js'));

var auth = require(path.join(INCLUDE_DIR, 'common', 'auth.js'));

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var Fs = require ( path.join(INCLUDE_DIR, 'common',
    'fileUtils.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var svcParams = ENV.SERVICES.email_fetch;
var rosSrvName = svcParams.ros_srv_name;


// Instantiate interface to rosbridge-websocket-server
var ros = new ROS({hostname: ENV.ROSBRIDGE.HOSTNAME, port: ENV.ROSBRIDGE.PORT,
  reconnect: true, onconnection: function(){
    // .
  }
});

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */


/**
 *  [Face-Detection] RAPP Platform front-end web service.
 *  <p> Serves requests for face_detection on given input image frame.</p>
 *
 *  @function face_detection
 *
 *  @param {Object} args - Service input arguments (object literal).
 *  @param {String} args.file_uri - System uri path of transfered (client)
 *    file, as declared in multipart/form-data post field. The file_uri is
 *    handled and forwared to this service, as input argument,
 *    by the HOP front-end server.
 *    Clients are responsible to declare this field in the multipart/form-data
 *    post field.
 *
 *  @returns {Object} response - JSON HTTPResponse Object.
 *    Asynchronous HTTP Response.
 *  @returns {Array} response.faces - An array of face-objects.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function svcImpl ( kwargs )
{
  var request = this;

  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
      auth.authRequest(request, svcParams.name, authSuccess, authFail);

      function authSuccess(user){
        kwargs = kwargs || {};
        var req = new interfaces.client_req();
        var response = new interfaces.client_res();
        var error = '';

        /***
         * Get argument values from request body and
         * create client_req object
         */
        try{
          svcUtils.parseReq(kwargs, req);
        }
        catch(e){
          error = "Service call arguments error";
          response.error = error;
          sendResponse( hop.HTTPResponseJson(response) );
          return;
        }
        /* ------------------------------------------ */

        if( ! req.email ){
          error = 'Empty \"email\" argument';
          response.error = error;
          sendResponse( hop.HTTPResponseJson(response) );
          return;
        }
        if( ! req.server ){
          error = 'Empty \"server\" argument';
          response.error = error;
          sendResponse( hop.HTTPResponseJson(response) );
          return;
        }
        // To Number input arguments conversions
        req.from_date = req.from_date;
        req.to_date = req.to_date;
        req.num_emails = req.num_emails;

        // Assign a unique identification key for this service request.
        var unqCallId = randStrGen.createUnique();

        var rosSvcReq = new interfaces.ros_req();
        rosSvcReq.email = req.email;
        rosSvcReq.password = req.passwd;
        rosSvcReq.server = req.server;
        rosSvcReq.port = req.port;
        rosSvcReq.requestedEmailStatus = req.email_status;
        rosSvcReq.fromDate = req.from_date;
        rosSvcReq.toDate = req.to_date;
        rosSvcReq.numberOfEmails = req.num_emails;


        /***
         * Declare the service response callback here!!
         * This callback function will be passed into the rosbridge service
         * controller and will be called when a response from rosbridge
         * websocket server arrives.
         */
        function callback(data){
          // Remove this call id from random string generator cache.
          randStrGen.removeCached( unqCallId );
          //console.log(data);
          // Craft client response using ros service ws response.
          var response = parseRosbridgeMsg( data );
          // Asynchronous response to client.
          sendResponse( hop.HTTPResponseJson(response) );
        }

        /***
         * Declare the onerror callback.
         * The onerror callack function will be called by the service
         * controller as soon as an error occures, on service request.
         */
        function onerror(e){
          // Remove this call id from random string generator cache.
          randStrGen.removeCached( unqCallId );
          // craft error response
          var response = new interfaces.client_res();
          response.error = svcUtils.ERROR_MSG_DEFAULT;
          // Asynchronous response to client.
          sendResponse( hop.HTTPResponseJson(response) );
        }

        ros.callService(rosSrvName, rosSvcReq,
          {success: callback, fail: onerror});

      }

      function authFail(error){
        var response = auth.responseAuthFailed();
        sendResponse(response);
      }

      /***
       *  Timeout this request. Return to client.
       */
      setTimeout(function(){
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
      }, svcParams.timeout);
      /* ----------------------------------------------- */

    }, this);
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
