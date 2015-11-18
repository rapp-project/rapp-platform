/*!
 * @file speech_detection_google.service.js
 * @brief Speech-Detection hop front-end service. Using google engine.
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

/* --------------------------< Load required modules >---------------------*/
var __modulePath = '../modules/';
var hop = require('hop');
var Fs = require( __modulePath + 'fileUtils.js' );
var RandStringGen = require ( __modulePath +
  'RandomStrGenerator/randStringGen.js' );
var ROS = require( __modulePath + '/RosBridgeJS/src/Rosbridge.js');
/*-------------------------------------------------------------------------*/

/* ------------< Load and set basic configuration parameters >-------------*/
var __DEBUG__ = false;
var user = process.env.LOGNAME;
var __configPath = '../config/';
var srvEnv = require( __configPath + 'env/hop-services.json' );
var pathsEnv = require( __configPath + 'env/paths.json' );
var __hopServiceName = 'speech_detection_google';
var __hopServiceId = null;
var __servicesCacheDir = Fs.resolve_path( pathsEnv.cache_dir_services );
var __serverCacheDir = Fs.resolve_path( pathsEnv.cache_dir_server );
/*-------------------------------------------------------------------------*/

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

var colors = {
  error:    '\033[1;31m',
  success:  '\033[1;31m',
  clear:    '\033[0m'
}

register_master_interface();


/*!
 * @brief Speech Detection (sphinx4) front-end Platform web-service
 * @param fileUrl
 * @param language
 * @param audio_source
 * @param words
 * @param sentences
 * @param grammar
 * @user
 */
service speech_detection_google({file_uri: '', audio_source: '', user: '',
  language: ''})
{
  /**For security reasons, if file_uri is not defined under the
   * server_cache_dir do not operate. HOP server stores the files under the
   * __serverCacheDir directory.
   */
  if( file_uri.indexOf(__serverCacheDir) === -1 )
  {
    var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
        " Abortion for security reasons.";
    postMessage( craft_slaveMaster_msg('log', errorMsg) );
    console.log(colors.error + '[Speech-Detection-Google]: ' + errorMsg +
      colors.clear);

    var response = {
      words: [],
      alternatives: [],
      error: errorMsg
    };
    return hop.HTTPResponseJson(response);
  }
  /* ----------------------------------------------------------------------- */

  // Assign a unique identification key for this service call.
  var unqCallId = randStrGen.createUnique();

  var startT = new Date().getTime();
  var execTime = 0;

  postMessage( craft_slaveMaster_msg('log', 'client-request {' + rosSrvName +
    '}') );
  var logMsg = 'Audio data stored at [' + file_uri + ']';
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var fileUrl = file_uri.split('/');
  var fileName = fileUrl[fileUrl.length -1];

  var cpFilePath = __servicesCacheDir + fileName.split('.')[0] + '-'  +
    unqCallId + '.' + fileName.split('.')[1];
  cpFilePath = Fs.resolve_path(cpFilePath);
  /* ---------------------------------------------------------------- */


  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.renameFile(file_uri, cpFilePath) == false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    var logMsg = 'Failed to rename file: [' + file_uri + '] --> [' +
      cpFilePath + ']';

    postMessage( craft_slaveMaster_msg('log', logMsg) );
    Fs.rmFile(file_uri);
    randStrGen.removeCached(unqCallId); // Dismiss the unique identity key
    var response = craft_error_response();
    return hop.HTTPResponseJson(response);
  }
  logMsg = 'Created copy of file ' + file_uri + ' at ' + cpFilePath;
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  /*-------------------------------------------------------------------------*/

  /**
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /**
       * These variables define information on service call.
       */
      var respFlag = false;
      var retClientFlag = false;
      var wsError = false;
      var retries = 0;
      /* --------------------------------------------------- */

      // Set Ros Service request arguments here.
      var args = {
        'filename': cpFilePath,
         'audio_type': audio_source,
         'user': user,
         'language': language
      };


      /**
       * Declare the service response callback here!!
       * This callback function will be passed into the rosbridge service
       * controller and will be called when a response from rosbridge
       * websocket server arrives.
       */
      function callback(data){
        respFlag = true;
        if( retClientFlag ) { return }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = craft_response( data );
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) )
        retClientFlag = true;
      }

      /**
       * Declare the onerror callback.
       * The onerror callack function will be called by the service
       * controller as soon as an error occures, on service request.
       */
      function onerror(e){
        respFlag = true;
        if( retClientFlag ) { return }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        // craft error response
        var response = craft_error_response();
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) )
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



/*!
 * @brief Crafts the form/format for the message to be returned
 * @param srvMsg Return message from ROS Service.
 * @return Message to be returned from the hop-service
 */
function craft_response(rosbridge_msg)
{
  var words = rosbridge_msg.words;
  var alternatives = rosbridge_msg.alternatives;
  var error = rosbridge_msg.error;
  var crafted_msg = { words: [], alternatives: [], error: '' };
  var logMsg = '';


  crafted_msg.words = words;
  for (var ii = 0; ii < alternatives.length; ii++)
  {
    crafted_msg.alternatives.push( alternatives[ii].s )
  }
  crafted_msg.error = error;
  logMsg = 'Returning to client.';

  if (error != '')
  {
    logMsg += ' ROS service [' + rosSrvName + '] error'
      ' ---> ' + error;
  }
  else
  {
    logMsg += ' ROS service [' + rosSrvName + '] returned with success'
  }
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return crafted_msg;
};



/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var errorMsg = 'RAPP Platform Failure';
  var crafted_msg = { words: [], alternatives: [], error: errorMsg };

  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return crafted_msg;
}


/*!
 * @brief Crafts ready to send, rosbridge message.
 *   Can be used by any service!!!!
 */
function craft_rosbridge_msg(args, service_name, id){

  var rosbrige_msg = {
    'op': 'call_service',
    'service': service_name,
    'args': args,
    'id': id
  };

  return rosbrige_msg;
}


function register_master_interface()
{
  // Register onexit callback function
  onexit = function(e){
    console.log("Service [%s] exiting...", __hopServiceName);
    var logMsg = "Received termination command. Exiting.";
    postMessage( craft_slaveMaster_msg('log', logMsg) );
  }

  // Register onmessage callback function
  onmessage = function(msg){
    if (__DEBUG__)
    {
      console.log("Service [%s] received message from master process",
        __hopServiceName);
      console.log("Msg -->", msg.data);
    };

    var logMsg = 'Received message from master process --> [' +
      msg.data + ']';
    postMessage( craft_slaveMaster_msg('log', logMsg) );

    exec_master_command(msg.data);
  }

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
  var msg = {
    name: __hopServiceName,
    id:   __hopServiceId,
    msgId: msgId,
    data: msg
  }
  return msg;
}


function isJson(str){
  try{
    JSON.parse(str);
  }
  catch(e){
    return false
  }
  return true
}
