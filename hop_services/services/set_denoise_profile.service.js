/*!
 * @file set_denoise_profile.service.js
 * @brief Set-Denoise-Profile hop front-end service.
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


/*--------------Load required modules-----------*/
var __modulePath = '../modules/';
var hop = require('hop');
var Fs = require( __modulePath + 'fileUtils.js' );
var RandStringGen = require( __modulePath +
  'RandomStrGenerator/randStringGen.js');
var RosSrvPool = require(__modulePath + 'ros/srvPool.js');
var RosParam = require(__modulePath + 'ros/rosParam.js')
/*----------------------------------------------*/

/*---------Sets required file Paths-------------*/
var __DEBUG__ = false;
var user = process.env.LOGNAME;
var __configPath = '../config/';
var srvEnv = require( __configPath + 'env/hop-services.json' );
var pathsEnv = require( __configPath + 'env/paths.json' )
var __hopServiceName = 'set_denoise_profile';
var __hopServiceId = null;
var __servicesCacheDir = Fs.resolve_path( pathsEnv.cache_dir_services );
var __serverCacheDir = Fs.resolve_path( pathsEnv.cache_dir_server );
/*----------------------------------------------*/


/*----<Load modules used by the service>----*/
var rosSrvName = srvEnv[__hopServiceName].ros_srv_name;
var rosParam = new RosParam({});
var rosSrvThreads = 0;

/* -------------------------< ROS service pool >-------------------------- */
var rosSrvPool = undefined;

rosParam.getParam_async('/rapp_audio_processing_threads', function(data){
  if(data > 0)
  {
    rosSrvThreads = data;
    rosSrvPool = new RosSrvPool(rosSrvName, rosSrvThreads);
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
 * @brief Set denoise profile (per-user) hop front-end service
 * @param noise_audio_fileUri
 * @param audio_file_type
 * @param user
 * @TODO Rename noise_audio_fileUri --> fileUrl
 */
service set_denoise_profile( {file_uri:'', audio_source:'', user:''}  )
{
  // For security reasons, if file_uri is not defined under the server_cache_dir
  // do not operate. HOP server stores the files under the __serverCacheDir
  // directory.
  if( file_uri.indexOf(__serverCacheDir) === -1 )
  {
    var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
      " Abortion for security reasons.";
    postMessage( craft_slaveMaster_msg('log', errorMsg) );
    console.log(errorMsg);
    var response = {
      error: errorMsg
    }
    return hop.HTTPResponseJson(response);
  }
  var startT = new Date().getTime();
  var execTime = 0;
  // Check if this service uses a threaPool and assign the relevant ros_service.
  if(rosSrvThreads) {var rosSrvCall = rosSrvPool.getAvailable();}
  else {var rosSrvCall = rosSrvName;}
  console.log(rosSrvCall);
  postMessage( craft_slaveMaster_msg('log', 'client-request {' + rosSrvCall +
    '}') );
  var logMsg = 'Audio data file stored at [' + file_uri + ']';
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqCallId = randStrGen.createUnique();
  var fileUrl = file_uri.split('/');
  var fileName = fileUrl[fileUrl.length -1];

  var cpFilePath = __servicesCacheDir + fileName.split('.')[0] + '-'  +
    unqCallId + '.' + fileName.split('.')[1];
  cpFilePath = Fs.resolve_path(cpFilePath);
  /* ---------------------------------------------------------------- */


  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.renameFile(file_uri, cpFilePath) == false)
  {
    if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
    //could not rename file. Probably cannot access the file. Return to client!
    var logMsg = 'Failed to rename file: [' + file_uri + '] --> [' +
      cpFilePath + ']';

    postMessage( craft_slaveMaster_msg('log', logMsg) );
    Fs.rmFile(file_uri);
    randStrGen.removeCached(unqCallId);
    var resp_msg = craft_error_response();
    return resp_msg;
  }
  logMsg = 'Created copy of file ' + file_uri + ' at ' + cpFilePath;
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  /*-------------------------------------------------------------------------*/

  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      var args = {
        'noise_audio_file': cpFilePath,
         'audio_file_type': audio_source,
         'user': user
      };

      var respFlag = false;
      var wsError = false;
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

          //console.log(event.value);
          Fs.rmFile(cpFilePath);
          respFlag = true; // Raise Response-Received Flag

          this.close(); // Close websocket
          rosWS = undefined; // Ensure deletion of websocket

          // Dismiss the unique call identity key for current client.
          randStrGen.removeCached( unqCallId );
          execTime = new Date().getTime() - startT;
          postMessage( craft_slaveMaster_msg('execTime', execTime) );
          var response = craft_response(event.value);
          sendResponse( hop.HTTPResponseJson(response));
        }
        rosWS.onerror = function(e){
          if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
          rosWS = undefined;
          wsError = true;

          var logMsg = 'Websocket' +
            'to rosbridge [ws//localhost:9090] got error...\r\n' + e;
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          Fs.rmFile(cpFilePath);
          var response = craft_error_response();
          sendResponse( hop.HTTPResponseJson(response));
          execTime = new Date().getTime() - startT;
          postMessage( craft_slaveMaster_msg('execTime', execTime) );
        }
      }
      catch(e){
        if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
        rosWS = undefined;
        wsError = true;

        var logMsg = 'ERROR: Cannot open websocket' +
          'to rosbridge [ws//localhost:9090]\r\n' + e;
        postMessage( craft_slaveMaster_msg('log', logMsg) );

        Fs.rmFile(cpFilePath);
        var response = craft_error_response();
        sendResponse( hop.HTTPResponseJson(response));
        execTime = new Date().getTime() - startT;
        postMessage( craft_slaveMaster_msg('execTime', execTime) );
        return;
      }
      /*------------------------------------------------------------------ */

      var retries = 0;

      // Set Timeout wrapping function
      function asyncWrap(){
        setTimeout( function(){

         if (respFlag || wsError) { return; }
         else{
           retries += 1;

           var logMsg = 'Reached rosbridge response timeout' + '---> [' +
             timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
             'Retry-' + retries;
           postMessage( craft_slaveMaster_msg('log', logMsg) );

           /* - Fail to receive message from rosbridge. Return to client */
           if (retries >= maxTries)
           {
             if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
             var logMsg = 'Reached max_retries [' + maxTries + ']' +
               ' Could not receive response from rosbridge...';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             Fs.rmFile(cpFilePath);

             rosWS.close();
             rosWS = undefined;
             //  Close websocket before return
             execTime = new Date().getTime() - startT;
             postMessage( craft_slaveMaster_msg('execTime', execTime) );
             var response = craft_error_response();
             sendResponse( hop.HTTPResponseJson(response));
             return;
           }

           if (rosWS != undefined) { rosWS.close(); }
           rosWS = undefined;

           /* --------------< Re-connect to Rosbridge >--------------*/
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
               Fs.rmFile(cpFilePath);

               respFlag = true;
               execTime = new Date().getTime() - startT;
               postMessage( craft_slaveMaster_msg('execTime', execTime) );
               var response = craft_response(event.value);
               sendResponse( hop.HTTPResponseJson(response));
               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket
             }
             rosWS.onerror = function(e){
               if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
               rosWS = undefined;
               wsError = true;

               var logMsg = 'Websocket' +
                 'to rosbridge [ws//localhost:9090] got error...\r\n' + e;
               postMessage( craft_slaveMaster_msg('log', logMsg) );

               Fs.rmFile(cpFilePath);
               var response = craft_error_response();
               sendResponse( hop.HTTPResponseJson(response));
               execTime = new Date().getTime() - startT;
               postMessage( craft_slaveMaster_msg('execTime', execTime) );
             }
           }
           catch(e){
             if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
             rosWS = undefined;
             wsError = true;

             var logMsg = 'ERROR: Cannot open websocket' +
               'to rosbridge --> [ws//localhost:9090]';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             Fs.rmFile(cpFilePath);

             execTime = new Date().getTime() - startT;
             postMessage( craft_slaveMaster_msg('execTime', execTime) );
             var response = craft_error_response();
             sendResponse( hop.HTTPResponseJson(response));
             return;
           }

         }
         /*--------------------------------------------------------*/
         asyncWrap();

       }, timeout);
     }
     asyncWrap();
/*============================================================================*/
   }, this );
};



/*!
 * @brief Crafts the form/format for the message to be returned
 * from set_denoise_profile hop-service.
 * @param srvMsg Return message from ROS Service.
 * return Message to be returned from the hop-service
 */
function craft_response(rosbridge_msg)
{
  var msg = JSON.parse(rosbridge_msg);
  var call_result = msg.result;
  var error = msg.values.error;
  var crafted_msg = { error: '' };
  var logMsg = '';
  //console.log(msg)

  if (call_result)
  {
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
  }
  else
  {
    logMsg = 'Communication with ROS service ' + rosSrvName +
      'failed. Unsuccesful call! Returning to client with error' +
      ' ---> RAPP Platform Failure';
    crafted_msg.error = 'RAPP Platform Failure';
  }

  //console.log(crafted_msg);
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  return crafted_msg;
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure';
  var crafted_msg = {error: errorMsg};

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
