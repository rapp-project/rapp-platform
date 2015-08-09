/*!
 * @file image_recognition.service.js
 * @brief Image Recognition hop front-end service.
 *
 */

"use strict";


// TODO -- Load PLATFORM parameters from JSON file
// TODO -- Load ROS-Topics/Services names from parameter server (ROS)

var __DEBUG__ = false;

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var module_path = '../utilities/js/'
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var Fs = require( module_path + 'fileUtils.js' );
var hop = require('hop');
var RandStringGen = require ( module_path + 'randStringGen.js' );
/*----------------------------------------------*/

var script_dir_path = __dirname;
var script_full_pah = __filename;
var rapp_image_recognition_path = script_dir_path +
  "/../../rapp_image_recognition/";
var data_path = rapp_image_recognition_path + "data/book_1/";
//console.log(data_path)

/*-----<Define Detect-Objects ROS service name>----*/
var ros_service_name = '/find_objects';
/*------------------------------------------------------*/

var __hopServiceName = 'detect_objects';
var __hopServiceId = null;
var __masterId = null;
var __storeDir = '~/.hop/cache/';

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/

/* -- Set timer values for websocket communication to rosbridge -- */
var timer_tick_value = 100 // ms
var max_time = 5000 // ms
var max_tries = 2
//var max_timer_ticks = 1000 * max_time / tick_value;
/* --------------------------------------------------------------- */

register_master_interface();


/*!
 * @brief Face Detection HOP Service Core.
 *
 * @param file_uri Path of uploaded image file. Returned by hop server.
 * @return Message response from faceDetection ROS Service.
 *
 */
service detect_objects ( {file_uri:'', limit: ''} )
{
  postMessage( craft_slaveMaster_msg('log', 'client-request') );

  var logMsg = 'Image stored at [' + file_uri + ']';
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqCallId = randStrGen.createUnique();
  var fileUrl = file_uri.split('/');
  var fileName = fileUrl[fileUrl.length -1];

  var cpFilePath = __storeDir + fileName.split('.')[0] + '-'  + unqCallId +
    '.' + fileName.split('.')[1];

  cpFilePath = Fs.resolve_path(cpFilePath);
  /* ---------------------------------------------------------------- */

  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.copyFile(file_uri, cpFilePath) == false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    var logMsg = 'Failed to rename file: [' + file_uri + '] --> [' +
      cpFilePath + ']';

    postMessage( craft_slaveMaster_msg('log', logMsg) );

    //Fs.rm_file_sync(file_uri);

    // Dismiss the unique identity key
    randStrGen.removeCached(unqCallId);
    var resp_msg = craft_error_response();
    return resp_msg;
  }
  /*-------------------------------------------------------------------------*/


  // Asynchronous Response. Implementation
  /*----------------------------------------------------------------- */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

     var files = [];
     /* Add parameters hardcoded */
     var names = [
       'cat', 'cow', 'goat', 'hen', 'pig', 'sheep', 'cock',
       'dog', 'goose', 'horse', 'rabbit', 'turkey'
     ];

     for (var n in names)
     {
       files.push(data_path + names[n] + '.jpg');
     }

     var args = {
       /* Image path to perform faceDetection, used as input to the
        *  Face Detection ROS Node Service
        */
       "fname": cpFilePath,
       "limit": parseInt(limit),
       "names": names,
       "files": files
     };

/*=============================TEMPLATE======================================================*/
      var rosbridge_connection = true;
      var respFlag = false;

      var rosbridge_msg = craft_rosbridge_msg(args, ros_service_name, unqCallId);

      /* ------ Catch exception while open websocket communication ------- */
      try{
        var rosWS = new WebSocket('ws://localhost:9090');

        // Register WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;

          var logMsg = 'Connection to rosbridge established';
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          this.send(JSON.stringify(rosbridge_msg));
        }
        // Implement WebSocket.onclose callback
        rosWS.onclose = function(){
          var logMsg = 'Connection to rosbridge closed';
          postMessage( craft_slaveMaster_msg('log', logMsg) );
        }
        // Implement WebSocket.message callback
        rosWS.onmessage = function(event){
          var logMsg = 'Received message from rosbridge';
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          Fs.rm_file_sync(cpFilePath);
          //console.log(event.value);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( unqCallId );
          sendResponse( resp_msg );
        }
      }
      catch(e){
        rosbridge_connection = false;

        var logMsg = 'ERROR: Cannot open websocket' +
          'to rosbridge --> [ws//localhost:9090]';
        postMessage( craft_slaveMaster_msg('log', logMsg) );

        Fs.rm_file_sync(cpFilePath);
        var resp_msg = craft_error_response();
        sendResponse( resp_msg );
        return;
      }
      /*------------------------------------------------------------------ */

      var timer_ticks = 0;
      var elapsed_time;
      var retries = 0;

      // Set Timeout wrapping function
      function asyncWrap(){
        setTimeout( function(){
         timer_ticks += 1;
         elapsed_time = timer_ticks * timer_tick_value;

         if (respFlag == true)
         {
           return
         }
         else if (respFlag != true && elapsed_time > max_time ){
           timer_ticks = 0;
           retries += 1;

           var logMsg = 'Reached rosbridge response timeout' +
             '---> [' + elapsed_time + '] ms ... Reconnecting to rosbridge.' +
             'Retry-' + retries;
           postMessage( craft_slaveMaster_msg('log', logMsg) );

           if (retries > max_tries) // Reconnected for max_tries times
           {
             var logMsg = 'Reached max_retries [' + max_tries + ']' +
               ' Could not receive response from rosbridge...';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             Fs.rm_file_sync(cpFilePath);
             var respMsg = craft_error_response();

             //  Close websocket before return
             rosWS.close();
             rosWS = undefined;
             sendResponse( respMsg );
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
               var logMsg = 'Received message from rosbridge';
               postMessage( craft_slaveMaster_msg('log', logMsg) );

               Fs.rm_file_sync(cpFilePath);
               var resp_msg = craft_response( event.value );

               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket
               respFlag = true;

               randStrGen.removeCached( unqCallId ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
               console.log("[Detect-Objects]: Returning to client");
             }
           }
           catch(e){
             rosbridge_connection = false;

             var logMsg = 'ERROR: Cannot open websocket' +
               'to rosbridge --> [ws//localhost:9090]';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             Fs.rm_file_sync(cpFilePath);
             console.log(e);
             var resp_msg = craft_error_response();
             sendResponse( resp_msg );
             return;
           }

         }
         /*--------------------------------------------------------*/
         asyncWrap(); // Recall timeout function

       }, timer_tick_value); //Timeout value is set at 100 ms.
     }
     asyncWrap();
/*==============================================================================================*/
   }, this );
};


/*!
 * @brief Crafts the form/format for the message to be returned
 * @param rosbridge_msg Return message from ROS Service.
 * return Message to be returned from service.
 */
function craft_response(rosbridge_msg)
{
  var msg = JSON.parse(rosbridge_msg);
  var found_names = msg.values.found_names;
  var found_centers = msg.values.found_centers;
  var found_scores = msg.values.found_scores;
  var call_result = msg.result;
  var result = msg.values.result;
  //var error = msg.values.error;

  var crafted_msg = {found_names:[], found_scores:[],
    error: '' };

  var logMsg = '';

  if (call_result)
  {
    for (var ii = 0; ii < found_names.length; ii++)
    {
      crafted_msg.found_names.push( found_names[ii] );
    }
    for (var ii = 0; ii < found_centers.length; ii++)
    {
      crafted_msg.objects.found_centers.push( found_centers[ii].point );
    }
    for (var ii = 0; ii < found_scores.length; ii++)
    {
      crafted_msg.found_scores.push( found_scores[ii] );
    }
    crafted_msg.error = "";

    //TODO Add error message field to ROS service msg!!!!
    //if (error != '')
    //{
      //logMsg += ' ROS service [' + ros_service_name + '] error'
        //' ---> ' + error;
    //}
    //else
    //{
      //logMsg += ' ROS service [' + ros_service_name + '] returned with success'
    //}
  }
  else
  {
    logMsg = 'Communication with ROS service ' + ros_service_name +
      'failed. Unsuccesful call! Returning to client with error' +
      ' ---> RAPP Platform Failure';
    crafted_msg.error = "RAPP Platform Failure";
  }

  //console.log(JSON.stringify(crafted_msg));
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  return JSON.stringify(crafted_msg)
};


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure';
  var crafted_msg = {found_names: [], found_centers: [], found_scores: [],
    error: errorMsg};

  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  return JSON.stringify(crafted_msg);
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
    case 2050:
      __masterId = data;
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
