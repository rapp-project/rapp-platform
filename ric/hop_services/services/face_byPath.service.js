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
var HopUtils = require( '../utilities/./HopServiceUtils.js' ); 

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
var hsu = new HopUtils();

/*!
 * @brief QR Node HOP Service Core.
 * @param _qrImage Image data in BINARY encoding/format.
 */
service face_byPath ( _imageURL, _clientParams )
{

  var randStr = randStrGen.createUnique();
  var fileName = "faceImage-" + randStr + ".png";

  console.log("\033[01;36mRequest for face_byPath service\033[0;0m");
  
   
  var imageStorePath = Fs.resolvePath( storePath + fileName );
  var args = {
    //"header": header,
    "imageFilename": imageStorePath   
  }; 

  /*-----<Request File and Stores >-----*/
  hsu.getFile( _imageURL, imageStorePath, _clientParams );
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
