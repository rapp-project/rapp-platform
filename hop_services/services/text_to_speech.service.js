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


/* ------------< Load and set basic configuration parameters >-------------*/
var __DEBUG__ = false;
var user = process.env.LOGNAME;
var module_path = '../modules/'
var config_path = '../config/';
var srvEnv = require( config_path + 'env/hop-services.json' )
var __hopServiceName = 'text_to_speech';
var __hopServiceId = null;
var __masterId = null;
var __cacheDir = '~/.hop/cache/services/';
var audioOutFormat = 'wav';
/* ----------------------------------------------------------------------- */

/* --------------------------< Load required modules >---------------------*/
var Fs = require( module_path + 'fileUtils.js' );
var fs = require('fs');
var hop = require('hop');
var RandStringGen = require ( module_path +
  'RandomStrGenerator/randStringGen.js' );
var RosSrvPool = require(module_path + 'ros/srvPool.js');
var RosParam = require(module_path + 'ros/rosParam.js')
/* ----------------------------------------------------------------------- */

var ros_service_name = srvEnv[__hopServiceName].ros_srv_name;
var rosParam = new RosParam({});
var rosSrvThreads = 0;  // Default is set at zero (0)

/* -------------------------< ROS service pool >-------------------------- */
var rosSrvPool = undefined;

rosParam.getParam_async('/rapp_text_to_speech_espeak_threads', function(data){
  if(data > 0)
  {
    rosSrvThreads = data;
    rosSrvPool = new RosSrvPool(ros_service_name, rosSrvThreads);
  }
});
/* ----------------------------------------------------------------------- */

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = srvEnv[__hopServiceName].timeout; // ms
var max_tries = srvEnv[__hopServiceName].retries;
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
  var startT = new Date().getTime();
  var execTime = 0;
  if(rosSrvThreads) {var rosSrvCall = rosSrvPool.getAvailable();}
  else {var rosSrvCall = ros_service_name;}
  console.log(rosSrvCall);
  postMessage( craft_slaveMaster_msg('log', 'client-request {' + rosSrvCall + '}') );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqCallId = randStrGen.createUnique();
  var audioOutPath = Fs.resolve_path(
    __cacheDir + unqCallId + '.' + audioOutFormat
    );
  /* ---------------------------------------------------------------- */

  // Asynchronous http response
  /*----------------------------------------------------------------- */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

     var args = {
       text: text,
       language: language,
       audio_output: audioOutPath
     };

      var respFlag = false;
      var rosbridge_msg = craft_rosbridge_msg(args, rosSrvCall, unqCallId)

      /**
       * ---- Catch exception on initiating websocket.
       *  -- Return to client immediately on exception thrown.
       */
      try{
        var rosWS = new WebSocket('ws://localhost:9090');

        // Register WebSocket.onopen callback
        rosWS.onopen = function(){
          var logMsg = 'Connection to rosbridge established';
          postMessage( craft_slaveMaster_msg('log', logMsg) );
          this.send(JSON.stringify(rosbridge_msg));
        }
        // Register WebSocket.onclose callback
        rosWS.onclose = function(){
          var logMsg = 'Connection to rosbridge closed';
          postMessage( craft_slaveMaster_msg('log', logMsg) );
        }
        // Register WebSocket.message callback
        rosWS.onmessage = function(event){
          if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
          var logMsg = 'Received message from rosbridge';
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          respFlag = true; // Raise Response-Received Flag

          this.close(); // Close websocket
          rosWS = undefined; // Ensure deletion of websocket

          // Dismiss the unique call identity key for current client.
          randStrGen.removeCached( unqCallId );
          execTime = new Date().getTime() - startT;
          postMessage( craft_slaveMaster_msg('execTime', execTime) );
          var response = craft_response(event.value, audioOutPath);
          sendResponse( response )
        }
      }
      catch(e){
        if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
        rosWS = undefined;

        var logMsg = 'ERROR: Cannot open websocket' +
          'to rosbridge [ws//localhost:9090]\r\n' + e;
        postMessage( craft_slaveMaster_msg('log', logMsg) );

        var response = craft_error_response();
        sendResponse( response );
        execTime = new Date().getTime() - startT;
        postMessage( craft_slaveMaster_msg('execTime', execTime) );
        return;
      }
      /*------------------------------------------------------------------ */

      var retries = 0;

      // Set Timeout wrapping function
      function asyncWrap(){
        setTimeout( function(){

         if (respFlag)
         {
           return;
         }
         else{
           retries += 1;

           var logMsg = 'Reached rosbridge response timeout' +
             '---> [' + timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
             'Retry-' + retries;
           postMessage( craft_slaveMaster_msg('log', logMsg) );

           /* - Fail to receive message from rosbridge. Return to client */
           if (retries >= max_tries)
           {
             if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
             var logMsg = 'Reached max_retries [' + max_tries + ']' +
               ' Could not receive response from rosbridge...';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             rosWS.close();
             rosWS = undefined;
             //  Close websocket before return
             execTime = new Date().getTime() - startT;
             postMessage( craft_slaveMaster_msg('execTime', execTime) );
             var response = craft_error_response();
             sendResponse( response );
             return;
           }

           if (rosWS != undefined)
           {
             rosWS.close();
           }
           rosWS = undefined;

           /* --------------< Re-open connection to the WebSocket >--------------*/
           try{
             rosWS = new WebSocket('ws://localhost:9090');

             /* -----------< Redefine WebSocket callbacks >----------- */
             rosWS.onopen = function(){
               var logMsg = 'Connection to rosbridge established';
               postMessage( craft_slaveMaster_msg('log', logMsg) );
               this.send(JSON.stringify(rosbridge_msg));
             }

             rosWS.onclose = function(){
               var logMsg = 'Connection to rosbridge closed';
               postMessage( craft_slaveMaster_msg('log', logMsg) );
             }

             rosWS.onmessage = function(event){
               if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
               var logMsg = 'Received message from rosbridge';
               postMessage( craft_slaveMaster_msg('log', logMsg) );

               //Remove the uniqueID so it can be reused
               randStrGen.removeCached( unqCallId );

               respFlag = true;
               execTime = new Date().getTime() - startT;
               postMessage( craft_slaveMaster_msg('execTime', execTime) );
               var response = craft_response(event.value, audioOutPath);
               sendResponse( response );
               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket
             }
           }
           catch(e){
             if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
             rosWS = undefined;
             //console.log(e);

             var logMsg = 'ERROR: Cannot open websocket' +
               'to rosbridge --> [ws//localhost:9090]';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             execTime = new Date().getTime() - startT;
             postMessage( craft_slaveMaster_msg('execTime', execTime) );
             var response = craft_error_response();
             sendResponse( response );
             return;
           }

         }
         /*--------------------------------------------------------*/
         asyncWrap(); // Recall timeout function

       }, timeout); //Timeout value is set at 100 ms.
     }
     asyncWrap();
/*============================================================================*/
   }, this );
};


/*!
 * @brief Crafts the form/format for the message to be returned
 * @param rosbridge_msg Return message from ROS Service.
 * return Message to be returned from service.
 */
function craft_response(rosbridge_msg, audioFilePath)
{
  var msg = JSON.parse(rosbridge_msg);
  var call_result = msg.result;
  var error = msg.values.error;
  var logMsg = '';
  //var crafted_msg = {data: '', error: ''};

  if (call_result)
  {
    logMsg = 'Returning to client.';

    if (error != '')
    {
      logMsg += ' ROS service [' + ros_service_name + '] error'
        ' ---> ' + error;
      //crafted_msg.error = error;
      var response = {error: error};
      return JSON.stringify(response);
    }
    else
    {
      logMsg += ' ROS service [' + ros_service_name + '] returned with success'
      //audioFile = Fs.read_file_sync(audioFilePath);
      //var data = {data: audioFile.data.toString('')};
      //console.log(data);
      var response = hop.HTTPResponseFile ( audioFilePath  );
      return response;
    }
  }
  else
  {
    logMsg = 'Communication with ROS service ' + ros_service_name +
      'failed. Unsuccesful call! Returning to client with error' +
      ' ---> RAPP Platform Failure';

    var response = {error: 'RAPP Platform Failure'};
    return JSON.stringify(response);
  }

  postMessage( craft_slaveMaster_msg('log', logMsg) );
  //console.log(crafted_msg)
  //return JSON.stringify(crafted_msg);
};


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var errorMsg = 'RAPP Platform Failure!'
  var crafted_msg = { error: errorMsg };

  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return JSON.stringify(crafted_msg);
}


/*!
 * @brief Crafts ready to send, rosbridge message.
 *   Can be used by any service!!!!
 */
function craft_rosbridge_msg(args, service_name, id)
{

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
    case 2050:
      __masterId = data;
      break;
    case 2065:
      __cacheDir = data;
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
