/*!
 * @file qr.service.js
 * @brief QR service running on Remote host.
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
var qrRosService = "/ric/ros_nodes/qr_detection_service";

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
service qrDetection ( _data )
{

  var randStr = randStrGen.createUnique();
  var fileName = "qrImage-" + randStr + ".jpg";
  var qrFoundMessage = false;

  console.log("\033[01;36mRequest for qrDetection service\033[0;0m");
  
   
  var qrImagePath = Fs.resolvePath( storePath + fileName );
  var args = {
    //"header": header,
    "imageFilename": qrImagePath //filenamePATH    
  }; 

  /*-----<Stores received image data>-----*/
  //Fs.writeBinFileSync( qrImagePath, _data );

  /*-----<Call QR ROS service through rosbridge>-----*/
  //var ros = new RosUtils();
  //ros.init_bridge('');
  //var returnMessage = ros.callService( qrRosService, args );
  var returnMessage = "I am a dummy message";

  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  return returnMessage; 
}
