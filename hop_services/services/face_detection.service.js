/*!
 * @file faceNode.service.js
 * @brief Face detection hop front-end service.
 *
 */

"use strict";


console.log('Initiated Face Detection front-end service');

// TODO -- Load PLATFORM parameters from JSON file
// TODO -- Load ROS-Topics/Services names from parameter server (ROS)


/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
var module_path = rapp_hop_path + 'utilities/js/'
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var Fs = require( module_path + 'fileUtils.js' );
var hop = require('hop');
var RandStringGen = require ( module_path + 'randStringGen.js' );
/*----------------------------------------------*/

/*-----<Define face-detection ROS service name>----*/
var rosService = '/rapp/rapp_face_detection/detect_faces';
/*------------------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/


/*!
 * @brief Face Detection HOP Service Core.
 *
 * @param fileUrl Path of uploaded image file. Returned by hop front-end
 * @return Message response from faceDetection ROS Node service.
 *
 */
service face_detection ( {fileUrl:''} )
{
  var randStr = randStrGen.createUnique();
  console.log("[face-detection]: Client Request");
  console.log('[face-detection]: Image stored at:', fileUrl);

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
 
     /* ------ Catch exception while open websocket communication ------- */
     try{
       var rosWS = new WebSocket('ws://localhost:9090');
     
       rosWS.onopen = function(){
         console.log('[face-detection]: Connection to rosbridge established');
         this.send(JSON.stringify(ros_srv_call));
       }
       rosWS.onclose = function(){
         console.log('[face-detection]: Connection to rosbridge closed');
       }
       rosWS.onmessage = function(event){
         console.log('[face-detection]: Received message from rosbridge');
         //console.log(event.value);
         var resp_msg = craft_response( event.value );
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
       var resp_msg = {faces_up_left: [], faces_down_right: [], error: "Platform is down!"};
       sendResponse( JSON.stringify(resp_msg) ); 
     }
     /*------------------------------------------------------------------ */

     function asyncWrap(){
       setTimeout( function(){
         if (respFlag != true){
           console.warn('[face-detection]: Connection timed out! rosWs = undefined');
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
               console.log('[face-detection]: Connection to rosbridge established');
               this.send(JSON.stringify(ros_srv_call));
             }

             rosWS.onclose = function(){
               console.log('[face-detection]: Connection to rosbridge closed');
             }

             rosWS.onmessage = function(event){
               console.log('[face-detection]: Received message from rosbridge');
               var resp_msg = craft_response( event.value ); 
               //console.log(resp_msg);
               this.close(); // Close the connection to the websocket
               rosWS = undefined; // Decostruct the websocket object
               respFlag = true;
               randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
             }
           }
           catch(e){
             console.log('[Error]: Cannot open websocket to rosbridge --> [ws//localhost:9090]' );
             console.log(etoString());
             var resp_msg = {faces_up_left: [], faces_down_right: [],  error: 'Platform is down!'};
             sendResponse( JSON.stringify(resp_msg) ); 
           }
           
           /*--------------------------------------------------------*/
           asyncWrap();
         }
       }, 3000); //Timeout value is set at 8 seconds
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
  var faces = JSON.parse(srvMsg).values;
  var result = JSON.parse(srvMsg).result;


  var craftedMsg = { faces_up_left:[], faces_down_right:[], error: '' };

  
  if (result == true)
  {
    for (var ii = 0; ii < faces.faces_up_left.length; ii++)
    {
      craftedMsg.faces_up_left.push( faces.faces_up_left[ii].point )
    }
    for (var ii = 0; ii < faces.faces_down_right.length; ii++)
    {
      craftedMsg.faces_down_right.push( faces.faces_down_right[ii].point )
    }   
    craftedMsg.error = '0'; 
  }
  else{
    craftedMsg.error = '1';
  }

  return JSON.stringify(craftedMsg)
  //return craftedMsg;
    /* Return JSON representation:
     *{ faces_up_left: [ { y:155, x:145, z:0} ],
     *   faces_down_right: [ { y:155, x:145, z:0} ],
     *   error: '0' }
     */
};

