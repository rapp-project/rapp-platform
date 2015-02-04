/*!
 * @file qr.service.js
 * @brief QR Detection Hop service running on Remote host.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";
/*----------------------------------------------*/
/*--------------Load required modules-----------*/
var Fs = require( /*rapp_hop_path +*/ "../utilities/./fileUtils.js" );
var ROSbridge = require( /*rapp_hop_path +*/ "../utilities/./rosbridge.js" );
var RandStringGen = require ( /*rapp_hop_path +*/ "../utilities/./randStringGen.js" );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var qrRosService = "/ric/ros_nodes/qr_detection_service";
/*--<Defines the directory where images received are stored>--*/
var storePath = "/home/klpanagi/hop_temps/"; 
/*---Create a RosBridge object to allow communication with rosbridge (Global)---*/
var rosbridge = new ROSbridge();
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
service qrDetection ( _file )
{
  rosbridge.connect(); //connect to rosbridge
  var randStr = randStrGen.createUnique();
  var fileName = "qrImage-" + randStr + ".jpg";
  var qrFoundMessage = false;
  console.log("[QR-Detection] Client Request");
   
  var qrImagePath = Fs.resolvePath( storePath + fileName );
  var args = {
   /* Image path to perform QR Detection, used as input to the 
     *  QR Detection ROS Node Service
     */
    "imageFilename": qrImagePath 
  }; 

  /*-----<Stores received image data>-----*/
  Fs.writeFileSync( qrImagePath, _file.data );
  /*-----<Call QR_Detection ROS service through rosbridge>-----*/
  //var rosbridge = new ROSbridge();
  //rosbridge.connect(); //connect to rosbridge
  var returnMessage = rosbridge.callServiceSync( qrRosService, args );
  rosbridge.close(); //disconnect from rosbridge
  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  return returnMessage; 
};
