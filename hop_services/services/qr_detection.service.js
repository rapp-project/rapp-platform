/*!
 * @file qr.service.js
 * @brief QR Detection hop front-end service.
 *
 */

"use strict";

console.log('Initiated QR Detection front-end service');


// TODO -- Load PLATFORM parameters from JSON file
// TODO -- Load ROS-Topics/Services names from parameter server (ROS)

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var Fs = require( rapp_hop_path + "utilities/fileUtils.js" );
var hop = require('hop');
var RandStringGen = require ( rapp_hop_path + "utilities/randStringGen.js" );
/*----------------------------------------------*/

/*-----<Defined Name of QR Node ROS service>----*/
var rosService = "/ric/ros_nodes/qr_detection_service";
/*------------------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/


/*!
 * @brief QR_Detection HOP Service Core.
 *
 * @param _file An Object literral that specifies a "data"
 *  property. Data must be raw_binary from buffer.
 *
 * @return Message response from qrDetection ROS Node service.
 */
service qr_detection ( {fileUrl:''} )
{
  var randStr = randStrGen.createUnique();
  console.log('[qr-detection]: Client Request');
  console.log('[qr-detection]: Image stored at:', fileUrl);

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
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
       /* Image path to perform faceDetection, used as input to the 
        *  Face Detection ROS Node Service
        */
       "imageFilename": fileUri_new
     };  

     var uniqueID = randStrGen.createUnique();
     var ros_srv_call = {
       'op': 'call_service',
        'service': rosService,
        'args': args,
        'id': uniqueID
     };

     var rosWS = new WebSocket('ws://localhost:9090');
     rosWS.onopen = function(){
       console.log('[qr-detection]: Connection to rosbridge established');
       this.send(JSON.stringify(ros_srv_call));
     }
     rosWS.onclose = function(){
       console.log('[qr-detection]: Connection to rosbridge closed');
     }
     rosWS.onmessage = function(event){
       console.log('[qr-detection]: Received message from rosbridge');
       //console.log(resp_msg);
       var resp_msg = craft_response(event.value);
       sendResponse( resp_msg );
       this.close();
       rosWS = undefined;
       respFlag = true;
       randStrGen.removeCached( uniqueID );
     }

     function asyncWrap(){
       setTimeout( function(){
         if (respFlag != true){
           console.warn('[qr-detection]: Connection timed out! rosWs = undefined');
           //sendResponse('Timeout');
           if (rosWS != undefined)
       {
         rosWS.close();
       }
       rosWS = undefined;
       /* --< Re-open connection to the WebSocket >--*/
       rosWS = new WebSocket('ws://localhost:9090');
       /* -----------< Redefine WebSocket callbacks >----------- */
       rosWS.onopen = function(){
         console.log('[qr-detection]: Connection to rosbridge established');
         this.send(JSON.stringify(ros_srv_call));
       }

       rosWS.onclose = function(){
         console.log('[qr-detection]: Connection to rosbridge closed');
       }

       rosWS.onmessage = function(event){
         console.log('[qr-detection]: Received message from rosbridge');
         //console.log(resp_msg);
         var resp_msg = craft_response(event.value); 
         sendResponse( resp_msg ); //Return response to client
         this.close(); // Close the connection to the websocket
         rosWS = undefined; // Decostruct the websocket object
         respFlag = true;
         randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
       }
       /*--------------------------------------------------------*/
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
  var qrCenters = JSON.parse(srvMsg).values.qr_centers;
  var result = JSON.parse(srvMsg).result;

  var craftedMsg = {qr_centers: [], error: ''};

  if (result == true){
    for (var ii = 0; ii < qrCenters.length; ii++)
    {
      craftedMsg.qr_centers.push(qrCenters[ii].point);
    }
    craftedMsg.error = '0';
  }
  else
  {
    // Return error index
    craftedMsg.error = '1';
  }

  return JSON.stringify(craftedMsg)
    /* Return JSON representation:
     */
}

