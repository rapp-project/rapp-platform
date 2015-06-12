/*!
 * @file speech_detection_sphinx4.service.js
 * @brief Speech-Detection hop front-end service.
 *
 * @bug Sometimes the websocket implementation from hop gets undefined before
 *   the onmessage event calls
 */


"use strict";

console.log('Initiated Speech Detection front-end service');


// TODO -- Load PLATFORM parameters from JSON file
// TODO -- Load ROS-Topics/Services names from parameter server (ROS)


/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var module_path = '../utilities/js/';
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
//var contents = require('../utilities/parameters.json');
var hop = require('hop');
var Fs = require( module_path + 'fileUtils.js' );
var RandStringGen = require ( module_path + 'randStringGen.js' );
/*----------------------------------------------*/

/*-----<Defined Name of QR Node ROS service>----*/
var ros_service_name = '/rapp/rapp_speech_detection_sphinx4/batch_speech_to_text';
/*------------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/

/* -- Set timer values for websocket communication to rosbridge -- */
var timer_tick_value = 100 // ms
var max_time = 15000 // ms
var max_tries = 2
//var max_timer_ticks = 1000 * max_time / tick_value;
/* --------------------------------------------------------------- */



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
service speech_detection_sphinx4( {fileUrl: '', language: '', audio_source: '', words: [], sentences: [], grammar: [], user: ''} ){

  console.log('[speech-detection-sphinx4]: Service invocation');
  console.log('[speech-detection-sphinx4]: Audio source file stored at:', fileUrl);
  //console.log('Words to search for:', words);
  //console.log('Sentences:', sentences);
  //console.log('Grammar:', grammar);

  // Create new unique identity key
  var unqExt = randStrGen.createUnique();
  var file = fileUrl.split('.');
  var fileUrl_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2];

  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.rename_file_sync(fileUrl, fileUrl_new) == false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    var resp_msg = craft_error_response(); 
    console.log("[speech-detection-sphinx4]: Returning to client");
    return JSON.stringify(resp_msg); 
  }
  /*-------------------------------------------------------------------------*/

  // Dismiss the unique identity key
  randStrGen.removeCached(unqExt);

  //var star_time = undefined;
  //var elapsed_time = undefined;

  // Asynchronous Response. Implementation
  return hop.HTTPResponseAsync(
    function( sendResponse ) { 

      /* ======== Create specific service arguments here ========= */ 
      var args = {
        'path': /*audioFileUrl*/fileUrl_new,
         'audio_source': audio_source,
         'words': JSON.parse(words),
         'sentences': JSON.parse(sentences),
         'grammar': JSON.parse(grammar),
         'language': language,
         'user': user
      };
      /* ========================================================= */
     
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
        console.error('[speech-detection-sphinx4] ERROR: Cannot open websocket to rosbridge' +  
          '--> [ws//localhost:9090]' );
        // Print exception 
        console.log(e);
        // Craft return to client message
        var resp_msg = craft_error_response();
        // Return to Client
        sendResponse( JSON.stringify(resp_msg) ); 
        console.log("[speech-detection-sphinx4]: Returning to client with error");
        return
      }
      /* ----------------------------------------------------------------- */
     
      /* ------- Add into a try/catch block to ensure safe access -------- */
      try{
        // Implement WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;
          console.log('[speech-detection-sphinx4]: Connection to rosbridge established');
          this.send(JSON.stringify(rosbridge_msg));
        }
        // Implement WebSocket.onclose callback
        rosWS.onclose = function(){
          console.log('[speech-detection-sphinx4]: Connection to rosbridge closed');
        }
        // Implement WebSocket.message callback
        rosWS.onmessage = function(event){
          console.log('[speech-detection-sphinx4]: Received message from rosbridge');
          //console.log(event.value);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket 
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( uniqueID ); 
          sendResponse( resp_msg );
          console.log("[speech-detection-sphinx4]: Returning to client");
        }
      }
      catch(e){
        rosbridge_connection = false;
        console.error('[speech-detection-sphinx4] --> ERROR: Cannot open websocket' + 
          'to rosbridge --> [ws//localhost:9090]' );
        console.log(e);
        var resp_msg = craft_error_response;
        sendResponse( JSON.stringify(resp_msg) ); 
        console.log("[speech-detection-sphinx4]: Returning to client with error");
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

         if (respFlag != true && elapsed_time > max_time ){
           timer_ticks = 0;
           retries += 1;

           console.log("[speech-detection-sphinx4]: Reached rosbridge response timeout" + 
             "---> [%s] ms ... Reconnecting to rosbridge. Retry-%s", 
             elapsed_time.toString(), retries.toString());

           if (retries > max_tries) // Reconnected for max_tries times
           {
             console.log("[speech-detection-sphinx4]: Reached max_retries (%s)" + 
               "Could not receive response from rosbridge... Returning to client",
               max_tries);
             var respMsg = craft_error_response();
             sendResponse( JSON.stringify(respMsg) );
             console.log("[speech-detection-sphinx4]: Returning to client with error");
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
             console.log('[speech-detection-sphinx4]: Connection to rosbridge established');
             this.send(JSON.stringify(rosbridge_msg));
             }

             rosWS.onclose = function(){
               console.log('[speech-detection-sphinx4]: Connection to rosbridge closed');
             }

             rosWS.onmessage = function(event){
               console.log('[speech-detection-sphinx4]: Received message from rosbridge');
               var resp_msg = craft_response( event.value ); 
               //console.log(resp_msg);
               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket 
               respFlag = true;
               randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
               console.log("[speech-detection-sphinx4]: Returning to client");
             }
           }
           catch(e){
             rosbridge_connection = false;
             console.error('[speech-detection-sphinx4] ---> ERROR: Cannot open websocket' + 
               'to rosbridge --> [ws//localhost:9090]' );
             console.log(e);
             var resp_msg = craft_error_response(); 
             sendResponse( JSON.stringify(resp_msg) ); 
             console.log("[speech-detection-sphinx4]: Returning to client with error");
             return
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
 * @param srvMsg Return message from ROS Service.
 * @return Message to be returned from the hop-service
 */
function craft_response(srvMsg)
{
  var words = JSON.parse(srvMsg).values.words;
  var result = JSON.parse(srvMsg).result;
  var error = JSON.parse(srvMsg).values.error;

  var craftedMsg = { words: [], error: '' };

  if(result)
  {
    for (var ii = 0; ii < words.length; ii++)
    {
      craftedMsg.words.push( words[ii] )
    }
    craftedMsg.error = error; 
  }
  else
  { 
    // Return error index!
    craftedMsg.error = "RAPP Platform Failure";
  }
  return JSON.stringify(craftedMsg)
  //return craftedMsg;
};



/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var craftedMsg = {words: [], error: 'RAPP Platform Failure'};
  return craftedMsg;
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


