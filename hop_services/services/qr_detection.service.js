/*!
 * @file qr_detection.service.js
 * @brief QR Detection hop front-end service.
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


/* --------------------------< Load required modules >---------------------*/
var __modulePath = '../modules/';
var hop = require('hop');
var Fs = require( __modulePath + 'fileUtils.js' );
var RandStringGen = require ( __modulePath +
  'RandomStrGenerator/randStringGen.js' );
var RosSrvPool = require(__modulePath + 'ros/srvPool.js');
var ROS = require( __modulePath + '/RosBridgeJS/src/Rosbridge.js');
/* ----------------------------------------------------------------------- */

/* ------------< Load and set basic configuration parameters >-------------*/
var __DEBUG__ = false;
var user = process.env.LOGNAME;
var __configPath = '../config/';
var srvEnv = require( __configPath + 'env/hop-services.json' )
var pathsEnv = require( __configPath + 'env/paths.json' )
var __hopServiceName = 'qr_detection';
var __hopServiceId = null;
var __servicesCacheDir = Fs.resolve_path( pathsEnv.cache_dir_services );
var __serverCacheDir = Fs.resolve_path( pathsEnv.cache_dir_server );
/* ----------------------------------------------------------------------- */

var rosSrvName = srvEnv[__hopServiceName].ros_srv_name;
var rosSrvThreads = 0;  // Default is set at zero (0)

/* -------------------------< ROS service pool >-------------------------- */
var rosSrvPool = undefined;

var ros = new ROS({hostname: '', port: '', reconnect: true, onconnection:
  function(){
    ros.getParam('/rapp_qr_detection_threads', function(data){
      if(data > 0)
    {
      rosSrvThreads = data;
      rosSrvPool = new RosSrvPool(rosSrvName, rosSrvThreads);
    }
    });
  }
});
/* ----------------------------------------------------------------------- */

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = srvEnv[__hopServiceName].timeout; // ms
var maxTries = srvEnv[__hopServiceName].retries;
/* ----------------------------------------------------------------------- */

var colors = {
  error:    '\033[1;31m',
  success:  '\033[1;31m',
  clear:    '\033[0m'
}

// Register communication interface with the master-process
register_master_interface();


/*!
 * @brief QR_Detection HOP Service Core.
 *
 * @param _file An Object literral that specifies a "data"
 *  property. Data must be raw_binary from buffer.
 *
 * @return Message response from qrDetection ROS Node service.
 */
service qr_detection ( {file_uri:''} )
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
    console.log(colors.error + '[QR-Detection]: ' + errorMsg + colors.clear);
    var response = {
      qr_centers: [],
      qr_messages: [],
      error: errorMsg
    };
    return hop.HTTPResponseJson(response);
  }

  // Assign a unique identification key for this service call.
  var unqCallId = randStrGen.createUnique();

  var startT = new Date().getTime();
  var execTime = 0;

  /** Check if this service uses a threaPool. If true, use the threadPool
   * module in order to use service with current minimum bandwidth.
   */
  if(rosSrvThreads) {var rosSrvCall = rosSrvPool.getAvailable();}
  else {var rosSrvCall = rosSrvName;}
  console.log(rosSrvCall);
  /* ------------------------------------------------------------------- */

  postMessage( craft_slaveMaster_msg('log', 'client-request {' + rosSrvCall +
    '}') );
  var logMsg = 'Image stored at [' + file_uri + ']';
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var fileUrl = file_uri.split('/');
  var fileName = fileUrl[fileUrl.length -1];

  var cpFilePath = __servicesCacheDir + fileName.split('.')[0] + '-'  + unqCallId +
    '.' + fileName.split('.')[1];
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

      // Define Ros Service request arguments here.
      var args = {
        "imageFilename": cpFilePath
      };

      /**
       * Define the service response callback here!!
       * This callback function will be passed into the rosbridge service
       * controller and will be called as long as the response from rosbridge
       * websocket server arrives.
       */
      function callback(data){
        respFlag = true;
        if(retClientFlag) {return}
        // Remove this call id from random string generator cache.
        randStrGen.removeCached(unqCallId);
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = craft_response(data);
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }
      /* -------------------------------------------------------- */

      /**
       * Add service calling into try/catch block in order to catch
       * rosbridge connection errors
       */
      try{
        ros.callService(rosSrvCall, args, callback);
      }
      catch(e){
        wsError = true;
        if(retClientFlag) {return}
        // Remove this call id from random string generator cache.
        randStrGen.removeCached(unqCallId);
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        // craft error response
        var response = craft_error_response();
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }
      /* -------------------------------------------------------- */

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
         if (respFlag || wsError) { return; }
         else{
           retries += 1;

           var logMsg = 'Reached rosbridge response timeout' + '---> [' +
             timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
             'Retry-' + retries;
           postMessage( craft_slaveMaster_msg('log', logMsg) );

           /**
            * Fail. Did not receive message from rosbridge.
            * Return to client.
            */
           if (retries >= maxTries)
           {
             retClientFlag = true;
             if(rosSrvThreads) {rosSrvPool.release(rosSrvCall);}
             var logMsg = 'Reached max_retries [' + maxTries + ']' +
               ' Could not receive response from rosbridge...';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             Fs.rmFile(cpFilePath);

             //  Close websocket before return
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
 * from the faceDetection hop-service.
 * @param srvMsg Return message from ROS Service.
 * return Message to be returned from the hop-service
 */
function craft_response(rosbridge_msg)
{
  var qrCenters = rosbridge_msg.qr_centers;
  var qrMessages = rosbridge_msg.qr_messages;
  var error = rosbridge_msg.error;
  var logMsg = '';

  var crafted_msg = {qr_centers: [], qr_messages: [], error: ''};

  for (var ii = 0; ii < qrCenters.length; ii++)
  {
    var qrPoint = { x: 0, y: 0};
    qrPoint.x = qrCenters[ii].point.x;
    qrPoint.y = qrCenters[ii].point.y;
    crafted_msg.qr_centers.push(qrPoint);
    crafted_msg.qr_messages.push(qrMessages[ii]);
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
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure';
  var crafted_msg = {qr_centers: [], qr_messages: [], error: errorMsg};

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


function craft_slaveMaster_msg(msgId, data)
{
  var msg = {
    name: __hopServiceName,
    id:   __hopServiceId,
    msgId: msgId,
    data: data
  };
  return msg;
}
