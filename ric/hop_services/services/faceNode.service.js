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
var RosUtils = require( /*rapp_hop_path +*/ "../utilities/./RosUtils.js" );
var RandStringGen = require ( /*rapp_hop_path +*/ "../utilities/./randStringGen.js" );
/*----------------------------------------------*/

/*-----<Defined Name of QR Node ROS service>----*/
var faceRosService = "/ric/face_detection_service";

/*--<Defines the directory where images received are stored>--*/
var storePath = "/home/klpanagi/hop_temps/"; 

/*---Initiatess Communication with RosBridge (Global)---*/
//var ros = new RosUtils();
//ros.init_bridge('');
/*------------------------------------------------------*/

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/


/*!
 * @brief QR Node HOP Service Core.
 * @param _qrImage Image data in BINARY encoding/format.
 */
service faceNode (_faceImage)
{

  var randStr = randStrGen.createUnique();
  var fileName = "faceImage-" + randStr + ".png";
  var faceFoundMessage = false;

  console.log("\033[01;36mRequest for faceNode service\033[0;0m");
  
   
  var faceImagePath = Fs.resolvePath( storePath + fileName );
  var args = {
    //"header": header,
    "imageFilename": faceImagePath //filenamePATH    
  }; 

  /*-----<Stores received image data>-----*/
  Fs.writeBinFileSync( faceImagePath, _faceImage );
  /*-----<Call QR ROS service through rosbridge>-----*/
  var ros = new RosUtils();
  ros.init_bridge('');
  var returnMessage = ros.callService( faceRosService, args );
  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  return returnMessage; 
}
