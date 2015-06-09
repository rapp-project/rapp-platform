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
var rosService = '/rapp/rapp_audio_processing/set_noise_profile';
/*----------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/


/*!
 * @brief Set denoise profile (per-user) hop front-end service
 * @param noise_audio_fileUri
 * @param audio_file_type
 * @param user
 */
service set_denoise_profile( {noise_audio_fileUri:'', audio_file_type:'', user:''}  )
{
  console.log('[set-denoise-profile]: Service invocation. Preparing response');
  console.log('[set-denoise-profile]: Audio source file stored at:', noise_audio_fileUri);

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqExt = randStrGen.createUnique();
  randStrGen.removeCached(unqExt);
  var file = noise_audio_fileUri.split('.');
  var fileUri_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2]
  Fs.rename_file_sync(noise_audio_fileUri, fileUri_new);
  /*----------------------------------------------------------------- */

  var respFlag = false;

  return hop.HTTPResponseAsync(
    function( sendResponse ) 
    { 
      var args = {
        'noise_audio_file': fileUri_new,
         'audio_file_type': audio_file_type,
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
        rosWS.onopen = function(){
          console.log('[set-denoise-profile]: Connection to rosbridge established');
          this.send(JSON.stringify(ros_srv_call));
        };
        rosWS.onclose = function(){
          console.log('[set-denoise-profile]: Connection to rosbridge closed');
        };
        rosWS.onmessage = function(event){
          console.log('[set-denoise-profile]: Received message from rosbridge');
          //console.log(event.value)
          var resp_msg = craft_response(event.value);
          sendResponse( resp_msg );
          this.close();
          rosWS = undefined;
          respFlag = true;
          randStrGen.removeCached( uniqueID );
        };
      }
      catch(e){
        console.log('[Error]: Cannot open websocket to rosbridge --> [ws//localhost:9090]' );
        console.log(e);
        var resp_msg = {error: "Platform is down!"};
        sendResponse( JSON.stringify(resp_msg) ); 
      }
      /*------------------------------------------------------------------ */


      function asyncWrap()
      {
        setTimeout( function()
        {
          if (respFlag != true)
          {
            console.log('[set-denoise-profile]: Connection timed out! rosWs = undefined');
            if (rosWS != undefined)
            {
              rosWS.close();
            };
            rosWS = undefined;

            /* --< Re-open connection to the WebSocket >--*/
            /* ------ Catch exception while open websocket communication ------- */
            try{
              rosWS = new WebSocket('ws://localhost:9090');
              /* -----------< Redefine WebSocket callbacks >----------- */
              rosWS.onopen = function(){
                console.log('[set-denoise-profile]: Connection to rosbridge established');
                this.send(JSON.stringify(ros_srv_call));
              }
              rosWS.onclose = function(){
                console.log('[set-denoise-profile]: Connection to rosbridge closed');
              }
              rosWS.onmessage = function(event){
                console.log('[set-denoise-profile]: Received message from rosbridge');
                //console.log(event.value);
                var resp_msg = craft_response(event.value); 
                sendResponse( resp_msg ); //Return response to client
                this.close(); // Close the connection to the websocket
                rosWS = undefined; // Decostruct the websocket object
                respFlag = true;
                randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
              }
            }
            catch(e){
              console.log('[Error]: Cannot open websocket to rosbridge --> [ws//localhost:9090]' );
              console.log(e);
              var resp_msg = {error: "Platform is down!"};
              sendResponse( JSON.stringify(resp_msg) ); 
            }
            /*--------------------------------------------------------*/
            asyncWrap();
          }
        }, 6000); //Timeout value is set at 10 seconds
      }
      asyncWrap();
    }, this ); // do not forget the <this> argument of hop.HTTResponseAsync 
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

  var craftedMsg = { error: '' };
  
  if (result == true)
  {
    craftedMsg.error = ''; 
  }
  else
  { 
    // Return error index!
    craftedMsg.error = '1';
  }

  return JSON.stringify(craftedMsg)
  //return craftedMsg;
}

