/*!
 * @file faceNode.service.js
 * @brief Face detection service running on Remote host.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var Fs = require( rapp_hop_path + "utilities/./fileUtils.js" );
var fs = require('fs');
//var ROSbridge = require("../utilities/./rosbridge.js");
var hop = require('hop');
//console.log(hop);
var RandStringGen = require ( rapp_hop_path +"utilities/./randStringGen.js" );
/*----------------------------------------------*/

/*-----<Defined Name of QR Node ROS service>----*/
var rosService = "/ric/face_detection_service";

/*---Initiatess Communication with RosBridge (Global)---*/
//var rosbridge = new ROSbridge();
//ros.init_bridge('');
/*------------------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/


/*!
 * @brief Face Detection HOP Service Core.
 *
 * @param _file An Object literral that specifies a "data"
 * property. Data must be raw_binary from buffer.
 *
 * @return Message response from faceDetection ROS Node service.
 *
 */

service face_detection ( {fileUrl:''} )
{
  var randStr = randStrGen.createUnique();
  console.log("[face_detection] Client Request");
  console.log('[face_detection]Image stored at:', fileUrl);

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqExt = randStrGen.createUnique();
  randStrGen.removeCached(unqExt);
  var file = fileUrl.split('.');
  var fileUri_new = file[0] + '.' + file[1] +  unqExt + '.' + file[2]
    fs.renameSync(fileUrl, fileUri_new);
  console.log


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
       console.log('Connection to rosbridge established');
       this.send(JSON.stringify(ros_srv_call));
     }
     rosWS.onclose = function(){
       console.log('Connection to rosbridge closed');
     }
     rosWS.onmessage = function(event){
       console.log('Received message from rosbridge');
       var resp_msg = event.value;
       sendResponse( resp_msg );
       console.log(resp_msg);
       this.close();
       rosWS = undefined;
       respFlag = true;
       randStrGen.removeCached( uniqueID );
     }

     function asyncWrap(){
       setTimeout( function(){
         if (respFlag != true){
           console.log('Connection timed out! rosWs = undefined');
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
         console.log('Connection to rosbridge established');
         this.send(JSON.stringify(ros_srv_call));
       }

       rosWS.onclose = function(){
         console.log('Connection to rosbridge closed');
       }

       rosWS.onmessage = function(event){
         console.log('Received message from rosbridge');
         var resp_msg = event.value; 
         sendResponse( resp_msg ); //Return response to client
         console.log(resp_msg);
         this.close(); // Close the connection to the websocket
         rosWS = undefined; // Decostruct the websocket object
         respFlag = true;
         randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
       }
       /*--------------------------------------------------------*/
       asyncWrap();
         }
       }, 8000); //Timeout value is set at 10 seconds
     }
     asyncWrap();

   }, this ); // do not forget the <this> argument of hop.HTTResponseAsync 
};


/*!
 * @brief Crafts the form/format for the message to be returned
 * from the faceDetection hop-service.
 * @param srvMsg Return message from ROS Service.
 * return Message to be returned from the hop-service
 */
function craftRetMsg(srvMsg)
{
  faces = srvMsg.values;

  var craftedMsg = { faces_up_left:[], faces_down_right:[] };
  for (var ii = 0; ii < faces.faces_up_left.length; ii++)
  {
    craftedMsg.faces_up_left.push( faces.faces_up_left[ii].point )
  }
  for (var ii = 0; ii < faces.faces_down_right.length; ii++)
  {
    craftedMsg.faces_down_right.push( faces.faces_down_right[ii].point )
  }

  return JSON.stringify(craftedMsg)
    /* Return JSON representation:
     *{ faces_up_left: [ { y:155, x:145, z:0} ],
     *   faces_down_right: [ { y:155, x:145, z:0} ] }
     */
};

