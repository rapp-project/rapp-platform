/*!
 * @file qr_detection.service.js
 * @brief QR Detection hop front-end service.
 *
 */

"use strict";

console.log('Initiated QR Detection front-end service');


// TODO -- Load PLATFORM parameters from JSON file
// TODO -- Load ROS-Topics/Services names from parameter server (ROS)
//
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

/*-----<Defined Name of QR Node ROS service>----*/
var ros_service_name = '/rapp/rapp_qr_detection/detect_qrs';
var hopServiceName = 'qr_detection'
var hopServiceId = null;
/*------------------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/

/* -- Set timer values for websocket communication to rosbridge -- */
var timer_tick_value = 100; // ms
var max_time = 1000; // ms
var max_tries = 3;
//var max_timer_ticks = 1000 * max_time / tick_value;
/* --------------------------------------------------------------- */

var workerId = null;
var masterId = null;

register_master_interface();
//var slaveMasterMsg = craft_slaveMaster_msg();


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
  postMessage( craft_slaveMaster_msg('log', 'client-request') );

  var logMsg = 'Image stored at [' + file_uri + ']';
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqExt = randStrGen.createUnique();
  var file = file_uri.split('.');
  var file_uri_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2];

  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.rename_file_sync(file_uri, file_uri_new) == false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    Fs.rm_file_sync(file_uri);
    // Dismiss the unique identity key
    randStrGen.removeCached(unqExt);
    var resp_msg = craft_error_response();
    return resp_msg;
  }
  /*-------------------------------------------------------------------------*/

  // Dismiss the unique identity key
  randStrGen.removeCached(unqExt);


  /*----------------------------------------------------------------- */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

     var args = {
       /* Image path to perform faceDetection, used as input to the
        *  Face Detection ROS Node Service
        */
       "imageFilename": file_uri_new
     };

/*=============================TEMPLATE======================================*/
      var rosbridge_connection = true;
      var respFlag = false;

      // Create a unique caller id
      var uniqueID = randStrGen.createUnique();
      var rosbridge_msg = craft_rosbridge_msg(args, ros_service_name, uniqueID);

      /* ------ Catch exception while open websocket communication ------- */
      try{
        var rosWS = new WebSocket('ws://localhost:9090');
      /* ----------------------------------------------------------------- */

      /* ------- Add into a try/catch block to ensure safe access -------- */
        // Register WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;
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
          var logMsg = 'Received message from rosbridge';
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          Fs.rm_file_sync(file_uri_new);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( uniqueID );
          sendResponse( resp_msg );
        }
      }
      catch(e){
        rosbridge_connection = false;

        var logMsg = 'ERROR: Cannot open websocket' +
          'to rosbridge --> [ws//localhost:9090]';

        // Update master and logger
        postMessage( craft_slaveMaster_msg('log', logMsg) );

        Fs.rm_file_sync(file_uri_new);
        console.log(e);
        var resp_msg = craft_error_response();
        sendResponse( resp_msg );
        return;
      }
      /*------------------------------------------------------------------ */

      var timer_ticks = 0;
      var elapsed_time = 0;
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

           if (retries >= max_tries) // Reconnected for max_tries times
           {
             var logMsg = 'Reached max_retries [' + max_tries + ']' +
               ' Could not receive response from rosbridge...';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             Fs.rm_file_sync(file_uri_new);
             var respMsg = craft_error_response();
             sendResponse( respMsg );
             //  Close websocket before return
             rosWS.close();
             rosWS = undefined;
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

               Fs.rm_file_sync(file_uri_new);
               var resp_msg = craft_response( event.value );

               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket
               respFlag = true;
               randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
             }
           }
           catch(e){
             rosbridge_connection = false;

             var logMsg = 'ERROR: Cannot open websocket' +
               'to rosbridge --> [ws//localhost:9090]';

             // Update master and logger
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             Fs.rm_file_sync(file_uri_new);
             console.log(e);
             var resp_msg = craft_error_response();
             sendResponse( resp_msg );
             return
           }

         }
         /*--------------------------------------------------------*/
         asyncWrap(); // Recall timeout function

       }, timer_tick_value); //Timeout value is set at 100 ms.
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
  var msg = JSON.parse(rosbridge_msg);
  var qrCenters = msg.values.qr_centers;
  var call_result = msg.result;
  var error = msg.values.error;
  //console.log(msg);

  var crafted_msg = {qr_centers: [], error: ''};

  var logMsg = '';
  if (call_result)
  {
    for (var ii = 0; ii < qrCenters.length; ii++)
    {
      crafted_msg.qr_centers.push(qrCenters[ii].point);
    }
    crafted_msg.error = error;
    logMsg = 'Returning to client.';

    if (error != '')
    {
      logMsg += ' ROS service [' + ros_service_name + '] error'
        ' ---> ' + error;
    }
    else
    {
      logMsg += ' ROS service [' + ros_service_name + '] returned with success'
    }
  }
  else
  {
    logMsg = 'Communication with ROS service ' + ros_service_name +
      'failed. Unsuccesful call! Returning to client with error' +
      ' ---> RAPP Platform Failure';
    crafted_msg.error = 'RAPP Platform Failure';
  }

  postMessage( craft_slaveMaster_msg('log', logMsg) );
  return JSON.stringify(crafted_msg)
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure!'
  var crafted_msg = {qr_centers: [], error: errorMsg};

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
  onexit = function(e){
    console.log("Service [%s] exiting...", hopServiceName);
    var logMsg = "Received termination command. Exiting.";
    postMessage( craft_slaveMaster_msg('log', logMsg) );
  }

  onmessage = function(msg){
    if (__DEBUG__)
    {
      console.log("Service [%s] received message from master process",
        hopServiceName);
      console.log("Msg -->", msg.data);
    };
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
      set_worker_id(data);
      break;
    case 2050:
      storeMasterId(data);
      break;
    default:
      break;
  }
}


function storeMasterId(id)
{
  masterId = id;
}


function set_worker_id(id)
{
  workerId = id;
};


function craft_slaveMaster_msg(msgId, msg)
{
  var msg = {
    name: hopServiceName,
    id:   workerId,
    msgId: msgId,
    data: msg
  }
  return msg;
}
