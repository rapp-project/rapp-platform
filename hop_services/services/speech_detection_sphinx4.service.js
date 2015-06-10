
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
var rosService = '/rapp/rapp_speech_detection_sphinx4/batch_speech_to_text';
/*------------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/

var timer_tick_value = 100 // ms
var max_time = 5000 // ms
//var max_timer_ticks = 1000 * max_time / tick_value;

service speech_detection_sphinx4( {fileUrl: '', language: '', audio_source: '', words: [], sentences: [], grammar: [], user: ''} ){

  console.log('[speech-detection]: Service invocation. Preparing response');
  console.log('[speech-detection]: Audio source file stored at:', fileUrl);
  //console.log('Words to search for:', words);
  //console.log('Sentences:', sentences);
  //console.log('Grammar:', grammar);

  // Create new unique identity key
  var unqExt = randStrGen.createUnique();
  var file = fileUrl.split('.');
  var fileUri_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2];

  if (Fs.rename_file_sync(fileUrl, fileUri_new) == false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    var resp_msg = {faces_up_left: [], faces_down_right: [], error: "Platform error!"};
    console.log("[face-detection]: Returning to client");
    return JSON.stringify(resp_msg); 
  }
  // Dismiss the unique identity key
  randStrGen.removeCached(unqExt);

  var rosbridge_connection = true;
  var respFlag = false;
  //var star_time = undefined;
  //var elapsed_time = undefined;

  // Asynchronous Response. Implementation
  return hop.HTTPResponseAsync(
    function( sendResponse ) { 
      
      var args = {
        'path': /*audioFileUrl*/fileUri_new,
        'audio_source': audio_source,
        'words': JSON.parse(words),
        'sentences': JSON.parse(sentences),
        'grammar': JSON.parse(grammar),
        'language': language,
        'user': user
      };

      var uniqueID = randStrGen.createUnique();
      var ros_srv_call = {
        'op': 'call_service',
        'service': rosService,
        'args': args,
        'id': uniqueID
      };

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
       var resp_msg = {words: [], error: "Platform error!"};
       // Return to Client
       sendResponse( JSON.stringify(resp_msg) ); 
       return
     }
     /* ----------------------------------------------------------------- */
     
     /* ------- Add into a try/catch block to ensure safe access -------- */
     try{
       // Implement WebSocket.onopen callback
       rosWS.onopen = function(){
         rosbridge_connection = true;
         console.log('[speech-detection-sphinx4]: Connection to rosbridge established');
         this.send(JSON.stringify(ros_srv_call));
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
       }
     }
     catch(e){
       rosbridge_connection = false;
       console.error('[speech-detection-sphinx4] --> ERROR: Cannot open websocket' + 
         'to rosbridge --> [ws//localhost:9090]' );
       console.log(e);
       var resp_msg = {words: [], error: "Platform error!"};
       sendResponse( JSON.stringify(resp_msg) ); 
       return
     }
     /*------------------------------------------------------------------ */
     var timer_ticks = 0;
     var elapsed_time;
     // Set Timeout wrapping function
     function asyncWrap(){
       setTimeout( function(){
         timer_ticks += 1;
         elapsed_time = timer_ticks * timer_tick_value;

         if (respFlag != true && elapsed_time > max_time){
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
             this.send(JSON.stringify(ros_srv_call));
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
             }
           }
           catch(e){
             rosbridge_connection = false;
             console.error('[speech-detection-sphinx4] ---> ERROR: Cannot open websocket' + 
               'to rosbridge --> [ws//localhost:9090]' );
             console.log(e);
             var resp_msg = {words: [],  error: 'Platform error!'};
             sendResponse( JSON.stringify(resp_msg) ); 
             return
           }

         }
         /*--------------------------------------------------------*/
         asyncWrap(); // Recall timeout function
         
       }, 100); //Timeout value is set at 100 ms.
     }
     asyncWrap();
   }, this ); 
};


/*!
 * @brief Crafts the form/format for the message to be returned
 * @param srvMsg Return message from ROS Service.
 *   return Message to be returned from the hop-service
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
    craftedMsg.error = "Platform error!";
  }
  return JSON.stringify(craftedMsg)
  //return craftedMsg;
};
