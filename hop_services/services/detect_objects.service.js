/*!
 * @file image_recognition.service.js
 * @brief Image Recognition hop front-end service.
 *
 */

"use strict";


console.log('Initiated Image Recognition front-end service');

// TODO -- Load PLATFORM parameters from JSON file
// TODO -- Load ROS-Topics/Services names from parameter server (ROS)


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


/*!
 * @brief Face Detection HOP Service Core.
 *
 * @param file_uri Path of uploaded image file. Returned by hop server.
 * @return Message response from faceDetection ROS Service.
 *
 */
service detect_objects ( {file_uri:'', limit: ''} )
{
  console.log("[Detect-Objects]: Client Request");
  console.log('[Detect-Objects]: Image stored at:', file_uri);
  console.log("[Detect-Objects]: Limit: ", limit);

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqExt = randStrGen.createUnique();
  var file = file_uri.split('.');
  var file_uri_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2];

  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.rename_file_sync(file_uri, file_uri_new) == false)
  {
    Fs.rm_file_sync(file_uri);
    // Dismiss the unique identity key
    randStrGen.removeCached(unqExt);
    //could not rename file. Probably cannot access the file. Return to client!
    var resp_msg = craft_error_response();
    console.log("[Detect-Objects]: Returning to client with error");
    return resp_msg;
  }

  /*-------------------------------------------------------------------------*/

  // Dismiss the unique identity key
  randStrGen.removeCached(unqExt);

  //var star_time = undefined;
  //var elapsed_time = undefined;

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

     //console.log(files);

     var args = {
       /* Image path to perform faceDetection, used as input to the
        *  Face Detection ROS Node Service
        */
       "fname": file_uri_new,
       "limit": parseInt(limit),
       "names": names,
       "files": files
     };

/*=============================TEMPLATE======================================================*/
      var rosbridge_connection = true;
      var respFlag = false;

      // Create a unique caller id
      var uniqueID = randStrGen.createUnique();
      var rosbridge_msg = craft_rosbridge_msg(args, ros_service_name, uniqueID);

      /* ------ Catch exception while open websocket communication ------- */
      try{
        var rosWS = new WebSocket('ws://localhost:9090');
      }
      catch(e){
        rosbridge_connection = false; // Could not open websocket to rosbridge websocket server
        console.error('[Detect-Objects] ERROR: Cannot open websocket to rosbridge' +
          '--> [ws//localhost:9090]' );
        Fs.rm_file_sync(file_uri_new);
        // Print exception
        console.log(e);
        // Craft return to client message
        var resp_msg = craft_error_response();
        // Return to Client
        sendResponse( resp_msg );
        console.log("[Detect-Objects]: Returning to client with error");
        return
      }
      /* ----------------------------------------------------------------- */

      /* ------- Add into a try/catch block to ensure safe access -------- */
      try{
        // Implement WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;
          console.log('[Detect-Objects]: Connection to rosbridge established');
          this.send(JSON.stringify(rosbridge_msg));
        }
        // Implement WebSocket.onclose callback
        rosWS.onclose = function(){
          console.log('[Detect-Objects]: Connection to rosbridge closed');
        }
        // Implement WebSocket.message callback
        rosWS.onmessage = function(event){
          console.log('[Detect-Objects]: Received message from rosbridge');
          Fs.rm_file_sync(file_uri_new);
          //console.log(event.value);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( uniqueID );
          sendResponse( resp_msg );
          console.log("[Detect-Objects]: Returning to client");
        }
      }
      catch(e){
        rosbridge_connection = false;
        console.error('[Detect-Objects] --> ERROR: Cannot open websocket' +
          'to rosbridge --> [ws//localhost:9090]' );
        Fs.rm_file_sync(file_uri_new);
        console.log(e);
        var resp_msg = craft_error_response();
        sendResponse( resp_msg );
        console.log("[Detect-Objects]: Returning to client with error");
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

           console.log("[Detect-Objects]: Reached rosbridge response timeout" + 
             "---> [%s] ms ... Reconnecting to rosbridge. Retry-%s",
             elapsed_time.toString(), retries.toString());

           if (retries > max_tries) // Reconnected for max_tries times
           {
             console.log("[Detect-Objects]: Reached max_retries (%s)" +
               "Could not receive response from rosbridge... Returning to client",
               max_tries);
             Fs.rm_file_sync(file_uri_new);
             var respMsg = craft_error_response();
             sendResponse( respMsg );
             console.log("[Detect-Objects]: Returning to client with error");
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
             console.log('[Detect-Objects]: Connection to rosbridge established');
             this.send(JSON.stringify(rosbridge_msg));
             }

             rosWS.onclose = function(){
               console.log('[Detect-Objects]: Connection to rosbridge closed');
             }

             rosWS.onmessage = function(event){
               console.log('[Detect-Objects]: Received message from rosbridge');
               Fs.rm_file_sync(file_uri_new);
               var resp_msg = craft_response( event.value ); 
               //console.log(resp_msg);
               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket 
               respFlag = true;
               randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
               console.log("[Detect-Objects]: Returning to client");
             }
           }
           catch(e){
             rosbridge_connection = false;
             console.error('[Detect-Objects] ---> ERROR: Cannot open websocket' + 
               'to rosbridge --> [ws//localhost:9090]' );
             Fs.rm_file_sync(file_uri_new);
             console.log(e);
             var resp_msg = craft_error_response(); 
             sendResponse( resp_msg ); 
             console.log("[Detect-Objects]: Returning to client with error");
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

  var crafted_msg = {found_names:[], found_scores:[],
    error: '' };
  
  if (call_result)
  {
    for (var ii = 0; ii < found_names.length; ii++)
    {
      crafted_msg.found_names.push( found_names[ii] );
    }
    //for (var ii = 0; ii < found_centers.length; ii++)
    //{
      //crafted_msg.objects.found_centers.push( found_centers[ii].point )
    //}   
    for (var ii = 0; ii < found_scores.length; ii++)
    {
      crafted_msg.found_scores.push( found_scores[ii] );
    }   
    crafted_msg.error = " "; 
  }
  else{
    crafted_msg.error = "RAPP Platform Failure";
  }
 
  //console.log(JSON.stringify(crafted_msg));
  return JSON.stringify(crafted_msg)
};


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var crafted_msg = {found_names: [], found_centers: [], found_scores: [], error: 'RAPP Platform Failure'};
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


