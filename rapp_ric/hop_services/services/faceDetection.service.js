/*!
 * @file faceNode.service.js
 * @brief Face detection service running on Remote host.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var Fs = require( /*rapp_hop_path +*/ "../utilities/./fileUtils.js" );
var ROSbridge = require("../utilities/./rosbridge.js");
var RandStringGen = require ( /*rapp_hop_path +*/ "../utilities/./randStringGen.js" );
/*----------------------------------------------*/

/*-----<Defined Name of QR Node ROS service>----*/
var faceRosService = "/ric/face_detection_service";

/*--<Defines the directory where images received are stored>--*/
var storePath = "/home/klpanagi/hop_temps/"; 

/*---Initiatess Communication with RosBridge (Global)---*/
var rosbridge = new ROSbridge();
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
 */
service faceDetection ( fileData )
{
  rosbridge.connect();
  var randStr = randStrGen.createUnique();
  var fileName = "faceImage-" + randStr + ".png";
  console.log("[faceDetection] Client Request");
  console.log('Is buffer?: ', Buffer.isBuffer(fileData));
  console.log("TypeOfData: ", typeof fileData);

  var faceImagePath = Fs.resolvePath( storePath + fileName );
  var args = {
    /* Image path to perform faceDetection, used as input to the 
     *  Face Detection ROS Node Service
     */
    "imageFilename": faceImagePath
  }; 
  /*-----<Stores received image data>-----*/
  Fs.writeFileSync( faceImagePath, fileData );
  /*-----<Call FaceDetection ROS service through rosbridge>-----*/
  //var rosbridge = new ROSbridge();
  //rosbridge.connect();
  var returnMessage = rosbridge.callServiceSync( faceRosService, args, 1 );
  rosbridge.close();
  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  var faces = craftRetMsg(returnMessage);
  return faces; 
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

