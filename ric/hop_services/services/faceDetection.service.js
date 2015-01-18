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
 * @param _data Image data in BINARY encoding/format.
 *
 * @return Message response from faceDetection ROS Node service.
 */
service faceDetection ( _data )
{
  rosbridge.connect();
  var randStr = randStrGen.createUnique();
  var fileName = "faceImage-" + randStr + ".png";

  console.log("[faceDetection] Client Request");
  
  var faceImagePath = Fs.resolvePath( storePath + fileName );
  var args = {
    /* Image path to perform faceDetection, used as input to the 
     *  Face Detection ROS Node Service
     */
    "imageFilename": faceImagePath
  }; 

  /*-----<Stores received image data>-----*/
  Fs.writeBinFileSync( faceImagePath, _data );

  /*-----<Call FaceDetection ROS service through rosbridge>-----*/
  //var rosbridge = new ROSbridge();
  //rosbridge.connect();
  var returnMessage = rosbridge.callServiceSync( faceRosService, args );
  rosbridge.close();
  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  return returnMessage; 
};
