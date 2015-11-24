/*!
 * @file text_to_speech.service.js
 * @brief Text To Speech hop front-end service.
 *
 */

/**
 *  MIT License (MIT)
 *
 *  Copyright (c) <2014> <Rapp Project EU>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  Authors: Konstantinos Panayiotou
 *  Contact: klpanagi@gmail.com
 *
 */


//"use strict";

var __modulePath = '../modules/';
var __configPath = '../config/';
var user = process.env.LOGNAME;
var __DEBUG__ = false;

/* --------------------------< Load required modules >---------------------*/
var hop = require('hop');
var Fs = require( __modulePath + 'fileUtils.js' );
var RandStringGen = require ( __modulePath +
  'RandomStrGenerator/randStringGen.js' );
var ROS = require( __modulePath + '/RosBridgeJS/src/Rosbridge.js');
/* ----------------------------------------------------------------------- */

/* ------------< Load and set basic configuration parameters >-------------*/
var srvEnv = require( __configPath + 'env/hop-services.json' );
var pathsEnv = require( __configPath + 'env/paths.json' );
var __hopServiceName = 'text_to_speech';
var __hopServiceId = null;
var __servicesCacheDir = Fs.resolve_path( pathsEnv.cache_dir_services );
var audioOutFormat = 'wav';
var basenamePrefix = 'tts_';
/* ----------------------------------------------------------------------- */

var rosSrvName = srvEnv[__hopServiceName].ros_srv_name;

// Initiate connection to rosbridge_websocket_server
var ros = new ROS({hostname: '', port: '', reconnect: true, onconnection:
  function(){
    // .
  }
});
/* ----------------------------------------------------------------------- */

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = srvEnv[__hopServiceName].timeout; // ms
var maxTries = srvEnv[__hopServiceName].retries;
/* ----------------------------------------------------------------------- */


register_master_interface();


/*!
 * @brief Face Detection HOP Service Core.
 *
 * @param file_uri Path of uploaded image file. Returned by hop server.
 * @return Message response from faceDetection ROS Service.
 *
 */
service text_to_speech ( {text: '', language: ''} )
{
  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();

  var startT = new Date().getTime();
  var execTime = 0;

  postMessage( craft_slaveMaster_msg('log', 'client-request {' + rosSrvName +
    '}') );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var audioOutPath = Fs.resolve_path(
    __servicesCacheDir + basenamePrefix + unqCallId + '.' + audioOutFormat
    );
  /* ---------------------------------------------------------------- */

  // Asynchronous http response
  /*----------------------------------------------------------------- */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /**
       * These variables define information on service request.
       */
      var respFlag = false;
      var retClientFlag = false;
      var wsError = false;
      var retries = 0;
      /* --------------------------------------------------- */

      var args = {
        text: text,
         language: language,
         audio_output: audioOutPath
      };


      /**
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
        //console.log(data);

        // Craft client response using ros service ws response.
        var response = craft_response( data, audioOutPath );
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }

      /**
       * Declare the onerror callback.
       * The onerror callack function will be called by the service
       * controller as soon as an error occures, on service request.
       */
      function onerror(e){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        var response = craft_error_response();
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }

      /* -------------------------------------------------------- */

      ros.callService(rosSrvName, args,
        {success: callback, fail: onerror});

      /**
       * Set Timeout wrapping function.
       * Polling in defined time-cycle. Catch timeout connections etc...
       */
      function asyncWrap(){
        setTimeout( function(){

         /**
          * If received message from rosbridge websocket server or an error
          * on websocket connection, stop timeout events.
          */
          if ( respFlag || wsError || retClientFlag ) { return; }

          retries += 1;

          var logMsg = 'Reached rosbridge response timeout' + '---> [' +
            timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
            'Retry-' + retries;
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          /**
           * Fail. Did not receive message from rosbridge.
           * Return to client.
           */
          if ( retries >= maxTries )
          {
            logMsg = 'Reached max_retries [' + maxTries + ']' +
              ' Could not receive response from rosbridge...';
            postMessage( craft_slaveMaster_msg('log', logMsg) );

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


/*!
 * @brief Crafts the form/format for the message to be returned
 * @param rosbridge_msg Return message from ROS Service.
 * return Message to be returned from service.
 */
function craft_response(rosbridge_msg, audioFilePath)
{
  var error = rosbridge_msg.error;
  var logMsg = '';
  var response = {payload: '', basename: '', encoding: '', error: ''};

  logMsg = 'Returning to client.';

  if (error !== '')
  {
    logMsg += ' ROS service [' + rosSrvName + '] error' +
      ' ---> ' + error;
    response.error = error;
  }
  else
  {
    logMsg += ' ROS service [' + rosSrvName + '] returned with success';
    if( (audioFile = Fs.read_file_sync(audioFilePath)) )
    {
      response.payload = audioFile.data.toString('base64');
      response.basename = audioFile.basename;
      response.encoding = 'base64';
      // Remove local file immediately.
      Fs.rmFile(audioFilePath);
    }
    else { response.error = 'RAPP Platform Failure'; }
  }
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  //console.log(crafted_msg)
  return response;
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var errorMsg = 'RAPP Platform Failure';
  var response = {payload: '', basename: '', encoding: '', error: errorMsg};

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

    exec_master_command(msg.data);
  };

  // On initialization inform master and append to log file
  var logMsg = "Initiated worker";
  postMessage( craft_slaveMaster_msg('log', logMsg) );
}


function exec_master_command(msg)
{
  var cmd = msg.cmdId;
  var data = msg.data;
  switch (cmd)
  {
    case 2055:  // Set worker ID
      __hopServiceId = data;
      break;
    case 2065:
      __servicesCacheDir = data;
      break;
    default:
      break;
  }
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
