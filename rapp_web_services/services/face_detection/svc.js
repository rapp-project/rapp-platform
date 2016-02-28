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

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.face_detection;
var SERVICE_NAME = svcParams.name;
var rosSrvName = svcParams.ros_srv_name;

var SERVICES_CACHE_DIR = ENV.PATHS.SERVICES_CACHE_DIR;
var SERVER_CACHE_DIR = ENV.PATHS.SERVER_CACHE_DIR;
/* ----------------------------------------------------------------------- */

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

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = svcParams.timeout; // ms
var maxTries = svcParams.retries;
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
  var req = new interfaces.client_req();
  var response = new interfaces.client_res();
  var error = '';

  kwargs = kwargs || {};
  for( var i in req ){
    req[i] = (kwargs[i] !== undefined) ? kwargs[i] : req[i];
  }
  if( ! req.file_uri ){
    error = 'No image file received';
    response.error = error;
    return hop.HTTPResponseJson(response);
  }
  // Workaround for bool and hop
  switch( req.fast ){
    case "True":
      req.fast = true;
      break;
    case "true":
      req.fast = true;
      break;
    case "False":
      req.fast = false;
      break;
    case "false":
      req.fast = false;
      break;
    case undefined:
      req.fast = false;
      break;
    default:
      break;
  }


  /***
   *  For security reasons, if file_uri is not defined under the
   *  server_cache_dir do not operate. HOP server stores the files under the
   *  SERVER_CACHE_DIR directory.
   */
  if( req.file_uri.indexOf(SERVER_CACHE_DIR) === -1 )
  {
    var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
        " Abortion for security reasons.";
    response.error = svcUtils.ERROR_MSG_DEFAULT;
    return hop.HTTPResponseJson(response);
  }
  /* ----------------------------------------------------------------------- */

  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();

  var startT = new Date().getTime();
  var execTime = 0;

  var cpFilePath = '';

  try{
    cpFilePath = svcUtils.cpInFile(req.file_uri, ENV.PATHS.SERVICES_CACHE_DIR,
      unqCallId);
  }
  catch(e){
    console.log(e);
    Fs.rmFile(req.file_uri);
    randStrGen.removeCached(unqCallId);

    response.error = svcUtils.ERROR_MSG_DEFAULT;
    return hop.HTTPResponseJson(response);
  }
  /*-------------------------------------------------------------------------*/


  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /***
       *  Status flags.
       */
      var respFlag = false;
      var retClientFlag = false;
      var wsError = false;
      var retries = 0;
      /* --------------------------------------------------- */

      // Fill Ros Service request msg parameters here.
      var rosSvcReq = new interfaces.ros_req();
      rosSvcReq.imageFilename = cpFilePath;
      rosSvcReq.fast = req.fast;


      /***
       * Declare the service response callback here!!
       * This callback function will be passed into the rosbridge service
       * controller and will be called when a response from rosbridge
       * websocket server arrives.
       */
      function callback(data){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = parseRosbridgeMsg( data );
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }

      /***
       * Declare the onerror callback.
       * The onerror callack function will be called by the service
       * controller as soon as an error occures, on service request.
       */
      function onerror(e){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        // craft error response
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }


      // Invoke ROS-Service request.
      ros.callService(rosSrvName, rosSvcReq,
        {success: callback, fail: onerror});

      /***
       * Set Timeout wrapping function.
       * Polling in defined time-cycle. Catch timeout connections etc...
       */
      function asyncWrap(){
        setTimeout( function(){

         /***
          * If received message from rosbridge websocket server or an error
          * on websocket connection, stop timeout events.
          */
          if ( respFlag || wsError || retClientFlag ) { return; }

          retries += 1;

          var logMsg = 'Reached rosbridge response timeout' + '---> [' +
            timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
            'Retry-' + retries;

          /***
           * Fail. Did not receive message from rosbridge.
           * Return to client.
           */
          if ( retries >= maxTries )
          {
            logMsg = 'Reached max_retries [' + maxTries + ']' +
              ' Could not receive response from rosbridge...';

            // Remove cached file. Release resources.
            Fs.rmFile(cpFilePath);

            execTime = new Date().getTime() - startT;

            var response = new interfaces.client_res();
            response.error = svcUtils.ERROR_MSG_DEFAULT;
            sendResponse( hop.HTTPResponseJson(response));
            retClientFlag = true;
            return;
          }
          /*--------------------------------------------------------*/
          asyncWrap();

        }, timeout);
      }
      asyncWrap();
      /*=================================================================*/
    }, this );
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
  var faces_up_left = rosbridge_msg.faces_up_left;
  var faces_down_right = rosbridge_msg.faces_down_right;
  var error = rosbridge_msg.error;
  var numFaces = faces_up_left.length;

  var logMsg = 'Returning to client';

  var response = new interfaces.client_res();

  if( error ){
    response.error = svcUtils.ERROR_MSG_DEFAULT;
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

registerSvc(svcImpl, svcParams);
