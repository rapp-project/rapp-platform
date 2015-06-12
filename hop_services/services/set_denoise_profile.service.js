/*!
 * @file set_denoise_profile.service.js
 * @brief Set-Denoise-Profile hop front-end service.
 *
 */

"use strict";

console.log('Initiated Set-Denoise_profile front-end service')


// TODO -- Load PLATFORM parameters from JSON file
// TODO -- Load ROS-Topics/Services names from parameter server (ROS)


/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var module_path = '../utilities/js/';
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var hop = require('hop');
var Fs = require( module_path + 'fileUtils.js' );
var RandStringGen = require( module_path + 'randStringGen.js');
/*----------------------------------------------*/

/*----<Load modules used by the service>----*/
var ros_service_name = '/rapp/rapp_audio_processing/set_noise_profile';
/*----------------------------------------------*/

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
 * @brief Set denoise profile (per-user) hop front-end service
 * @param noise_audio_fileUri
 * @param audio_file_type
 * @param user
 * @TODO Rename noise_audio_fileUri --> fileUrl
 */
service set_denoise_profile( {noise_audio_fileUri:'', audio_file_type:'', user:''}  )
{
  console.log('[set-denoise-profile]: Service invocation. Preparing response');
  console.log('[set-denoise-profile]: Audio source file stored at:', noise_audio_fileUri);

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqExt = randStrGen.createUnique();
  randStrGen.removeCached(unqExt);
  var fileUrl = noise_audio_fileUri;
  var file = noise_audio_fileUri.split('.');
  var fileUrl_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2]

  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.rename_file_sync(fileUrl, fileUrl_new) == false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    var resp_msg = craft_error_response(); 
    console.log("[set-denoise-profile]: Returning to client");
    return JSON.stringify(resp_msg); 
  }
  /*-------------------------------------------------------------------------*/

  return hop.HTTPResponseAsync(
    function( sendResponse ) { 

      var args = {
        'noise_audio_file': fileUrl_new,
         'audio_file_type': audio_file_type,
         'user': user
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
        console.error('[set-denoise-profile] ERROR: Cannot open websocket to rosbridge' +  
          '--> [ws//localhost:9090]' );
        // Print exception 
        console.log(e);
        // Craft return to client message
        var resp_msg = craft_error_response();
        // Return to Client
        sendResponse( JSON.stringify(resp_msg) ); 
        console.log("[set-denoise-profile]: Returning to client with error");
        return
      }
      /* ----------------------------------------------------------------- */
     
      /* ------- Add into a try/catch block to ensure safe access -------- */
      try{
        // Implement WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;
          console.log('[set-denoise-profile]: Connection to rosbridge established');
          this.send(JSON.stringify(rosbridge_msg));
        }
        // Implement WebSocket.onclose callback
        rosWS.onclose = function(){
          console.log('[set-denoise-profile]: Connection to rosbridge closed');
        }
        // Implement WebSocket.message callback
        rosWS.onmessage = function(event){
          console.log('[set-denoise-profile]: Received message from rosbridge');
          //console.log(event.value);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket 
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( uniqueID ); 
          sendResponse( resp_msg );
          console.log("[set-denoise-profile]: Returning to client");
        }
      }
      catch(e){
        rosbridge_connection = false;
        console.error('[set-denoise-profile] --> ERROR: Cannot open websocket' + 
          'to rosbridge --> [ws//localhost:9090]' );
        console.log(e);
        var resp_msg = craft_error_response;
        sendResponse( JSON.stringify(resp_msg) ); 
        console.log("[set-denoise-profile]: Returning to client with error");
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

           console.log("[set-denoise-profile]: Reached rosbridge response timeout" + 
             "---> [%s] ms ... Reconnecting to rosbridge. Retry-%s", 
             elapsed_time.toString(), retries.toString());

           if (retries > max_tries) // Reconnected for max_tries times
           {
             console.log("[set-denoise-profile]: Reached max_retries (%s)" + 
               "Could not receive response from rosbridge... Returning to client",
               max_tries);
             var respMsg = craft_error_response();
             sendResponse( JSON.stringify(respMsg) );
             console.log("[set-denoise-profile]: Returning to client with error");
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
             console.log('[set-denoise-profile]: Connection to rosbridge established');
             this.send(JSON.stringify(rosbridge_msg));
             }

             rosWS.onclose = function(){
               console.log('[set-denoise-profile]: Connection to rosbridge closed');
             }

             rosWS.onmessage = function(event){
               console.log('[set-denoise-profile]: Received message from rosbridge');
               var resp_msg = craft_response( event.value ); 
               //console.log(resp_msg);
               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket 
               respFlag = true;
               randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
               console.log("[set-denoise-profile]: Returning to client");
             }
           }
           catch(e){
             rosbridge_connection = false;
             console.error('[set-denoise-profile] ---> ERROR: Cannot open websocket' + 
               'to rosbridge --> [ws//localhost:9090]' );
             console.log(e);
             var resp_msg = craft_error_response(); 
             sendResponse( JSON.stringify(resp_msg) ); 
             console.log("[set-denoise-profile]: Returning to client with error");
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
 * from set_denoise_profile hop-service.
 * @param srvMsg Return message from ROS Service.
 * return Message to be returned from the hop-service
 */
function craft_response(srvMsg)
{
  // Service invocation success index
  var result = JSON.parse(srvMsg).result;
  var error = JSON.parse(srvMsg).values.error;

  var craftedMsg = { error: '' };
  
  if (result)
  {
    craftedMsg.error = error; 
  }
  else
  { 
    // Return error index!
    craftedMsg.error = "RAPP Platform Failure";
  }

  //console.log(craftedMsg);
  return JSON.stringify(craftedMsg)
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var craftedMsg = {error: 'RAPP Platform Failure'};
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


