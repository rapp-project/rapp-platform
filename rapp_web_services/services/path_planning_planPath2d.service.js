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
 * [Path-planning-plann-path] RAPP Platform front-end web service.
 *
 *  @author Wojciech Dudek
 *  @mail wojciechsbox@gmail.com
 *  @copyright Rapp Project EU 2015
 */



var __DEBUG__ = false;

var hop = require('hop');
var path = require('path');

var __includeDir = path.join(__dirname, '..', 'modules');
var __configDir = path.join(__dirname, '..', 'config');

var Fs = require( path.join(__includeDir, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(__includeDir, 'common',
    'randStringGen.js') );

var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );

var srvEnv = require( path.join(__configDir, 'env', 'hop-services.json') );
var pathsEnv = require( path.join(__configDir, 'env', 'paths.json') );

/* ------------< Load and set global configuration parameters >-------------*/
var __hopServiceName = 'path_planning_planPath2d';
var __hopServiceId = null;
var __servicesCacheDir = Fs.resolvePath( pathsEnv.cache_dir_services );
var __serverCacheDir = Fs.resolvePath( pathsEnv.cache_dir_server );
/* ----------------------------------------------------------------------- */

var rosSrvName = srvEnv[__hopServiceName].ros_srv_name;

// Initiate connection to rosbridge_websocket_server
var ros = new ROS({hostname: '', port: '', reconnect: true, onconnection:
  function(){
    // .
  }
});

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = srvEnv[__hopServiceName].timeout; // ms
var maxTries = srvEnv[__hopServiceName].retries;
/* ----------------------------------------------------------------------- */

var colors = {
  error:    String.fromCharCode(0x1B) + '[1;31m',
  success:  String.fromCharCode(0x1B) + '[1;32m',
  ok:       String.fromCharCode(0x1B) + '[34m',
  yellow:   String.fromCharCode(0x1B) + '[33m',
  clear:    String.fromCharCode(0x1B) + '[0m'
};


// Register communication interface with the master-process
register_master_interface();



/**
 *  [Path-planning-plann-path] RAPP Platform front-end web service.
 *  <p> Serves requests for path_planning_plann_path on given input image frame.</p>
 *
 *  @function path_planning_plann_path
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

service path_planning_planPath2d({user_name: '', map_name: '', robot_type: '', algorithm: '', start: '', goal: ''})
{
//
//check if map_exist
//chceck if algorithm exist
//



  /* ----------------------------------------------------------------------- */

  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();

  var startT = new Date().getTime();
  var execTime = 0;

  postMessage( craft_slaveMaster_msg('log', 'client-request {' + rosSrvName +
    '}') );
  var logMsg = 'Using map: ['+map_name+'], that belongs to user: ['+user_name+'], planning using: ['+algorithm+'] algorithm';
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  

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
 /*
 # Contains path to the desired map
string user_name
# Contains path to the desired map
string map_name
# Contains type of the robot. It is required to determine it's parameters (footprint etc.)
string robot_type
# Contains path planning algorithm name
string algorithm
# Contains start pose of the robot
geometry_msgs/PoseStamped start
# Contains goal pose of the robot
geometry_msgs/PoseStamped goal
*/
      // Fill Ros Service request msg parameters here.
      var PoseStamped_start = //{//new ROSLIB.Message({
       start;
      //};//);
      var PoseStamped_goal = //{//new ROSLIB.Message({
        goal;
     // };//);

      var args = {
        user_name: user_name,
        map_name: map_name,
        robot_type: robot_type,
        algorithm: algorithm,
        start: PoseStamped_start,
        goal: PoseStamped_goal,
      };


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
        var response = craft_response( data );
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
        //Fs.rmFile(cpFilePath);
        // craft error response
        var response = craft_error_response();
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }


      // Invoke ROS-Service request.
      ros.callService(rosSrvName, args,
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
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          /***
           * Fail. Did not receive message from rosbridge.
           * Return to client.
           */
          if ( retries >= maxTries )
          {
            logMsg = 'Reached max_retries [' + maxTries + ']' +
              ' Could not receive response from rosbridge...';
            postMessage( craft_slaveMaster_msg('log', logMsg) );

            // Remove cached file. Release resources.
            Fs.rmFile(cpFilePath);

            execTime = new Date().getTime() - startT;
            postMessage( craft_slaveMaster_msg('execTime', execTime) );

            var response = craft_error_response();
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

uint8 plan_found
string error_message

# if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal
geometry_msgs/PoseStamped[] path



 */
function craft_response(rosbridge_msg)
{
  var plan_found = rosbridge_msg.plan_found;
  var error_message = rosbridge_msg.error_message;
  var path = rosbridge_msg.path;

  var response = {
    plan_found: plan_found,
    error_message: error_message,
    path: path
  };

  if (error_message !== '')
  {
    logMsg += ' ROS service [' + rosSrvName + '] error' +
      ' ---> ' + error_message;
  }
  else
  {
    logMsg += ' ROS service [' + rosSrvName + '] returned with success';
  }
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return response;
}


/***
 *  Craft service error response object. Used to return to client when an
 *  error has been occured, while processing client request.
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure';

  var response = {
    plan_found: [],
    error_message: errorMsg,
    path: []
  };

  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return response;
}


/***
 *  Register interface with the main hopjs process. After registration
 *  this worker service can communicate with the main hopjs process through
 *  websockets.
 *
 *  The global scoped postMessage is used in order to send messages to the main
 *  process.
 *  Furthermore, the global scoped onmessage callback function declares the
 *  handler for incoming messages from the hopjs main process.
 *
 *  Currently log messages are handled by the main process.
 */
function register_master_interface()
{
  // Register onexit callback function
  onexit = function(e){
    console.log("Service [%s] exiting...", __hopServiceName);
    var logMsg = "Received termination command. Exiting.";
    postMessage( craft_slaveMaster_msg('log', logMsg) );
  };

  // Register onmessage callback function
  onmessage = function(msg){
    if (__DEBUG__)
    {
      console.log("Service [%s] received message from master process",
        __hopServiceName);
      console.log("Msg -->", msg.data);
    }

    var logMsg = 'Received message from master process --> [' +
      msg.data + ']';
    postMessage( craft_slaveMaster_msg('log', logMsg) );

    var cmd = msg.data.cmdId;
    var data = msg.data.data;
    switch (cmd)
    {
      case 2055:  // Set worker ID
        __hopServiceId = data;
        break;
      default:
        break;
    }
  };

  // On initialization inform master and append to log file
  var logMsg = "Initiated worker";
  postMessage( craft_slaveMaster_msg('log', logMsg) );
}


/***
 *  Returns master-process comm msg literal.
 */
function craft_slaveMaster_msg(msgId, msg)
{
  var _msg = {
    name: __hopServiceName,
    id:   __hopServiceId,
    msgId: msgId,
    data: msg
  };
  return _msg;
}
