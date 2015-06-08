
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
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
var module_path = rapp_hop_path + 'utilities/js/';
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


service speech_detection_sphinx4( {fileUrl: '', language: '', audio_source: '', words: [], sentences: [], grammar: [], user: ''} ){
  console.log('[speech-detection]: Service invocation. Preparing response');
  console.log('[speech-detection]: Audio source file stored at:', fileUrl);
  //console.log('Words to search for:', words);
  //console.log('Sentences:', sentences);
  //console.log('Grammar:', grammar);

  /* --< Perform renaming on the received file. Add uniqueId value> --- */
  var unqExt = randStrGen.createUnique();
  randStrGen.removeCached(unqExt);
  var file = fileUrl.split('.');
  var fileUri_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2];
  Fs.rename_file_sync(fileUrl, fileUri_new);
  /*----------------------------------------------------------------- */

  var respFlag = false;
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

      try{
        var rosWS = new WebSocket('ws://localhost:9090');
        rosWS.onopen = function(){
          console.log('[speech-detection]: Connection to rosbridge established');
          this.send(JSON.stringify(ros_srv_call));
        }
        rosWS.onclose = function(){
          console.log('[speech-detection]: Connection to rosbridge closed');
        }
        rosWS.onmessage = function(event){
          console.log('[speech-detection]: Received message from rosbridge');
          //console.log(event.value);
          var resp_msg = craft_response(event.value);
          this.close();
          rosWS = undefined;
          respFlag = true;
          randStrGen.removeCached( uniqueID );
          sendResponse( resp_msg );
        }
      }
      catch(e){
        console.log('[Error]: Cannot open websocket to rosbridge --> [ws//localhost:9090]' );
        console.log(e);
        var resp_msg = {words: [], error: "Platform is down!"};
        sendResponse( JSON.stringify(resp_msg) ); 
      }

      function asyncWrap(){
        setTimeout( function(){
          if (respFlag != true){
            console.warn('[speech-detection]: Connection timed out! rosWs = undefined');
            //sendResponse('Timeout');
            if (rosWS != undefined)
            {
              rosWS.close();
            }

            rosWS = undefined;
            /* --< Re-open connection to the WebSocket >--*/
            try{
              rosWS = new WebSocket('ws://localhost:9090');
              /* -----------< Redefine WebSocket callbacks >----------- */
              rosWS.onopen = function(){
                console.log('[speech-detection]: Connection to rosbridge established');
                this.send(JSON.stringify(ros_srv_call));
              }

              rosWS.onclose = function(){
                console.log('[speech-detection]: Connection to rosbridge closed');
              }

              rosWS.onmessage = function(event){
                console.log('[speech-detection]: Received message from rosbridge');
                //console.log(event.value);
                var resp_msg = craft_response(event.value); 
                sendResponse( resp_msg ); //Return response to client
                this.close(); // Close the connection to the websocket
                rosWS = undefined; // Decostruct the websocket object
                respFlag = true;
                randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
              }
              /*--------------------------------------------------------*/
            }
            catch(e){
              console.log('[Error]: Cannot open websocket to rosbridge --> [ws//localhost:9090]' );
              console.log(e);
              var resp_msg = {words: [], error: "Platform is down!"};
              sendResponse( JSON.stringify(resp_msg) ); 
            }
              
            asyncWrap();
          }
        }, 8000); //Timeout value is set at 8 seconds
      }
      asyncWrap();

    }, this ); 
};


/*!
 * @brief Crafts the form/format for the message to be returned
 * from the faceDetection hop-service.
 * @param srvMsg Return message from ROS Service.
 * return Message to be returned from the hop-service
 */
function craft_response(srvMsg)
{
  var words = JSON.parse(srvMsg).values.words;
  var result = JSON.parse(srvMsg).result;

  var craftedMsg = { words: [], error: '' };
  
  if (result == true)
  {
    for (var ii = 0; ii < words.length; ii++)
    {
      craftedMsg.words.push( words[ii] )
    }
      craftedMsg.error = '0'; 
  }
  else
  { 
    // Return error index!
    craftedMsg.error = '1';
  }

  return JSON.stringify(craftedMsg)
  //return craftedMsg;
};
