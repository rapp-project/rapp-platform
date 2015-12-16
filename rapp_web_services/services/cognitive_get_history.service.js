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
 * [Cognitive-get-history] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 *  Basic Components (modules)
 *
 *      hop = require('hop').
 *        Import and use hopjs functionalities.
 *        For more information visit:
 *            https://github.com/manuel-serrano/hop
 *
 *      RosBridgeJS.js
 *        Use this module to achieve communication with ROS-framework.
 *        This module integrates a service controller to connect to
 *        the rosbridge-websocket-server.
 *        For more information on rosbridge-websocket-server visit:
 *            http://wiki.ros.org/rosbridge_suite
 *
 *        For more information on the RosBridgeJS module visit:
 *            https://github.com/klpanagi/RosBridgeJS
 *
 *      RandStrGenerator.js
 *        Random string generator class to generate cached unique
 *        identity keys. Used to generate a unique id for each client
 *        service request.
 *
 */

var __DEBUG__ = false;

/***
 * Import hopjs module.
 */
var hop = require('hop');

/***
 * Import Nodejs Path module.
 */
var path = require('path');

/***
 * Set the include directory path. This directory contains imported modules
 * used while developing HOP Web Services.
 */
var __includeDir = path.join(__dirname, '..', 'modules');

/***
 * Set the config directory path. Configuration files are stored under this
 * directory.
 */
var __configDir = path.join(__dirname, '..', 'config');

/***
 * Import the RandomStrGenerator module.
 */
var RandStringGen = require ( path.join(__includeDir, 'common',
    'randStringGen.js') );

/***
 * Import the RosBridgeJS module.
 */
var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );

/***
 * Load hop-services.json file. This file contains hop-services basic
 * configuration parameters.
 * @constant
 */
var srvEnv = require( path.join(__configDir, 'env', 'hop-services.json') );


/*** ------------< Load and set global configuration parameters >----------*/

/***
 * This variable declares the HOP Service name. The name must be the same
 * as the service definition name, below.
 *
 * ### IMPLEMENTATION. SET __hopServiceName VALUE ###
 */
var __hopServiceName = 'cognitive_get_history';
var __hopServiceId = null;
/* ----------------------------------------------------------------------- */

/***
 * In case of communication with ROS-Service(s), the relevant
 * ROS-Service name has to be declared into the hop-services.json file.
 *
 * Here we read the value for the ROS-Service name.
 */
var rosSrvName = srvEnv[__hopServiceName].ros_srv_name;

/***
 * Initiate connection to rosbridge_websocket_server.
 *
 * By default, it tries to connect to ws://localhost:9090.
 * This is the default URL the rosbridge-websocket-server listens to.
 */
var ros = new ROS({hostname: '', port: '', reconnect: true, onconnection:
  function(){
    // .
  }
});

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;  // Random-String-Generator string length.

/***
 * Initiate RandomStringGenerator. This ensures initiation of the identities
 * cache. Initiate under the global scope.
 */
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */

/* ------< Set timer values for websocket communication to rosbridge> ----- */

/***
 * Load the timeout value for this service. This value is stored into
 * the hop-services.json file under the relevant field for this service.
 * The timeout value is used to timeout client service request.
 */
var timeout = srvEnv[__hopServiceName].timeout; // ms
var maxTries = srvEnv[__hopServiceName].retries;
/* ----------------------------------------------------------------------- */

var color = {
  error:    String.fromCharCode(0x1B) + '[1;31m',
  success:  String.fromCharCode(0x1B) + '[1;32m',
  ok:       String.fromCharCode(0x1B) + '[34m',
  yellow:   String.fromCharCode(0x1B) + '[33m',
  clear:    String.fromCharCode(0x1B) + '[0m'
};


/***
 * Register master-process interface.
 * Logging is handled by the master-process. Logging messages are passed to
 * master-process. The master-process then forwards received logging messages
 * to the logger.
 *
 * Each service has its own logging streams.
 *
 */
register_master_interface();


 /**
 *  [Cognitive-get-history] RAPP Platform front-end web service.
 *  Handles requests for cognitive_get_history query.
 *
 *  @function cognitive_get_history
 *
 *  @param {Object} args - Service input arguments (literal).
 *  @param {String} args.query - Recursive query.
 *
 *
 *  @returns {Object} response - JSON HTTPResponse Object.
 *    Asynchronous HTTP Response.
 *  @returns {Array} response.results - Query results.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 *
 */
service cognitive_get_history ( {user:'', from_time: 0, to_time: 0} )
{
  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();  
  /***
   * Asynchronous http response.
   *
   * @constant
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /***
       *  Status flags.
       *
       *  @constant
       *
       *  LEAVE THIS BLOCK AS IS!!!
       *===========================*/
      var respFlag = false;
      var retClientFlag = false;
      var wsError = false;
      var retries = 0;
      /*===========================*/

      /**
       * Fill Ros Service request msg parameters here.
       * @example
       *   var args = {param1: '', param2: ''}
       *
       * 
       */
      console.log(typeof up_to_time)
      var args = {
        //imageFilename: cpFilePath
        username: user,
        fromTime: parseInt(from_time),
        toTime: parseInt(to_time)
      };


      /***
       * Declare the ROS-Service response callback here!!
       * This callback function will be passed into the rosbridge service
       * controller and will be called when a response from rosbridge
       * websocket server arrives.
       */
      function callback(data){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
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
       *
       */
      function onerror(e){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // craft error response
        var response = craft_error_response();
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }


      /***
       * Call ROS-Service.
       * Input arguments:
       *   - rosSrvName: The name of the ROS-Service to call</li>
       *   - args: ROS-Service request message arguments</li>
       *   - objLiteral: An object literal to declare the onsuccess and
       *        onerror callbacks.
       *      { success: <onsuccess_callback>,fail: <onerror_callback> }
       *
       */
      ros.callService(rosSrvName, args,
        {success: callback, fail: onerror});

      /***
       * Executes this function when the timeout value has been reached.
       * If the maxTries has been reached, terminate this service request
       * and immediately return to client.
       *
       * LEAVE THIS BLOCK AS IS!!!!! OR EXTEND ITS FUNCTIONALITIES!!!
       *
       *=====================================================================*/
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
            randStrGen.removeCached( unqCallId );

            logMsg = 'Reached max_retries [' + maxTries + ']' +
              ' Could not receive response from rosbridge...';
            postMessage( craft_slaveMaster_msg('log', logMsg) );

            var response = craft_error_response();

            // Asynchronous client response.
            sendResponse( hop.HTTPResponseJson(response));
            retClientFlag = true;
            return;
          }
          /*--------------------------------------------------------*/
          asyncWrap();

        }, timeout);
      }
      asyncWrap();
      /*=====================================================================*/
    }, this );
}




/**
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *
 *  
 */
function craft_response(rosbridge_msg)
{
  var trace = rosbridge_msg.trace;
  var success = rosbridge_msg.success;
  var error = rosbridge_msg.error;
  
  var records = rosbridge_msg.records; 
  
  var logMsg = 'Returning to client';
  
  var CognitiveExercisePerformanceRecordsMsg = {
    timestamp: 0,
    test: "",
    type: "",
    subtype: "",
    difficulty: "",
    score: 0,
    meanScoreForTypeUpToNow: 0
  };
  
  var response = {
    CognitiveExercisePerformanceRecordsMsg: [],
    error: error
  };

  response.CognitiveExercisePerformanceRecordsMsg = records;

  /***
   * Report to logger the error message if an error has been occured
   * on ROS-Service request.
   */
  if (error !== '')
  {
    logMsg += ' ROS service [' + rosSrvName + '] error' +
      ' ---> ' + error;
  }
  else
  {
    logMsg += ' ROS service [' + rosSrvName + '] returned with success';
  }

  /***
   * Forward logging message to the master-process
   */
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return response;
}


/**
 *  Craft service error response object. Used to return to client when an
 *  error has been occured, while processing client request.
 *
 *  @returns {Object} response - Response Object.
 *
 *  
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure';

  /***
   * Craft Web Service response object literal here !!!
   *
   * 
   */
  var CognitiveExercisePerformanceRecordsMsg = {
    timestamp: 0,
    test: "",
    type: "",
    subtype: "",
    difficulty: "",
    score: 0,
    meanScoreForTypeUpToNow: 0
  };  
  var response = {
    CognitiveExercisePerformanceRecordsMsg: [],
    error: errorMsg
  };


  /***
   * Report to logger through the master-process.
   */
  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return response;
}


/**
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
 *
 *
 *  LEAVE THIS FUNCTION AS IS!!!!! OR EXTEND ITS FUNCTIONALITIES!!!
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


/**
 *  Returns master-process comm msg literal.
 *
 *  @param {String} msgId
 *  @param {String} msg - The logging message to forward to the logger.
 *
 *  @returns (Object) master-slave-msg
 *
 *  LEAVE THIS FUNCTION AS IS!!!!!
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
