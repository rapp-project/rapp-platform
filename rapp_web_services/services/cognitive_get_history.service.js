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

var hop = require('hop');
var path = require('path');

var ENV = require( path.join(__dirname, '..', 'env.js') );

var __includeDir = path.join(__dirname, '..', 'modules');
var __configDir = path.join(__dirname, '..', 'config');

var RandStringGen = require ( path.join(__includeDir, 'common',
    'randStringGen.js') );

var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );


/*** ------------< Load and set global configuration parameters >----------*/
var __hopServiceName = 'cognitive_get_history';
var __hopServiceId = null;
/* ----------------------------------------------------------------------- */

var rosSrvName = ENV.SERVICES[__hopServiceName].ros_srv_name;

// Initiate communication with rosbridge-websocket-server
var ros = new ROS({hostname: ENV.ROSBRIDGE.HOSTNAME, port: ENV.ROSBRIDGE.PORT,
  reconnect: true, onconnection: function(){
    // .
  }
});

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;  // Random-String-Generator string length.
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = ENV.SERVICES[__hopServiceName].timeout; // ms
var maxTries = ENV.SERVICES[__hopServiceName].retries;
/* ----------------------------------------------------------------------- */


register_master_interface();


/**
 *  [Cognitive-get-history] RAPP Platform front-end web service.
 *  Handles requests for cognitive_get_history query.
 *
 *  @function cognitive_get_history
 *
 *  @param {Object} args - Service input arguments (literal).
 *  @param {String} args.user - Username.
 *  @param {Number} args.from_time - User's history from-time.
 *  @param {Number} args.to_time - User's history to-time.
 *
 *
 *  @returns {Object} response - JSON HTTPResponse Object.
 *    Asynchronous HTTP Response.
 *  @returns {Array} response.records - User's history trace on Cognitive
 *    Exercises.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 *
 */
service cognitive_get_history ( {user:'', from_time: 0, to_time: 0, test_type: ''} )
{
  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();

  /***
   * Asynchronous http response.
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /***
       *  Status flags.
       *===========================*/
      var respFlag = false;
      var retClientFlag = false;
      var wsError = false;
      var retries = 0;
      /*===========================*/

      var args = {
        username: user,
        fromTime: parseInt(from_time),
        toTime: parseInt(to_time),
        testType: test_type
      };


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


      ros.callService(rosSrvName, args,
        {success: callback, fail: onerror});


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




/***
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *  @returns {Object} response - Response Object.
 *
 */
function craft_response(rosbridge_msg)
{
  var trace = rosbridge_msg.trace;
  var success = rosbridge_msg.success;
  var error = rosbridge_msg.error;
  var recordsPerClass = rosbridge_msg.recordsPerTestType;
  var testClasses = rosbridge_msg.testCategories;

  var response = {
    records: {},
    error: error
  };

  var logMsg = 'Returning to client';

  for( var ii = 0; ii < testClasses.length; ii++ ){
    try{
      response.records[testClasses[ii].toLowerCase()] =
        recordsPerClass[ii].records;
    }
    catch(e){
      response.records[testClasses[ii].toLowerCase()] = [];
    }
  }

  if (error !== '')
  {
    logMsg += ' ROS service [' + rosSrvName + '] error' +
      ' ---> ' + error;
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
 *
 *  @returns {Object} response - Response Object.
 *
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure';

  var response = {
    records: {},
    error: errorMsg
  };


  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return response;
}


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
