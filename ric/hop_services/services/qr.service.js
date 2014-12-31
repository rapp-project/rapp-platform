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
var Fs = require( rapp_hop_path + "utilities/./fileUtils.js" );
var RosUtils = require( rapp_hop_path + "utilities/./RosUtils.js" );
/*----------------------------------------------*/

/*-----<Defined Name of QR Node ROS service>----*/
var qrRosService = "/ric/ros_nodes/qr_detection_service";

/*--<Defines the directory where images received are stored>--*/
var storePath = "/home/klpanagi/hop_temps/"; 

/*---Initiatess Communication with RosBridge (Global)---*/
//var ros = new RosUtils();
//ros.init_bridge('');
/*------------------------------------------------------*/
var cache = new Object();
var cacheIdx = -1;

   
function randomStringGen(_length, _charsArray) {
  var chars = _charsArray || "0123456789ABCDEFGHIJKLMNOPQRSTUVWXTZabcdefghiklmnopqrstuvwxyz";
  var string_length = _length || 5;
  var randomString = "";
  for (var i=0; i<string_length; i++) {
    var rnum = Math.floor(Math.random() * chars.length);
    randomString += chars.substring(rnum,rnum+1);
  }
  return randomString;
}

/*!
 * @brief QR Node HOP Service Core.
 * @param _qrImage Image data in BINARY encoding/format.
 */
service qrNode (_qrImage)
{

  var ros = new RosUtils();
  ros.init_bridge('');
  var randStr = "";
  cacheIdx = cacheIdx + 1;
  //randStr = randomStringGen(5);
  //while( cache[randStr] != undefined )
  do{
    randStr = randomStringGen(5);

  }
  while ( cache[randStr.toString()] != undefined );
  cache[randStr.toString()] = true; //true means exist.
  console.log(cache);

  var fileName = "qrImage-" + randStr + ".jpg";
  var qrFoundMessage = false;

  console.log("\033[01;36mRequest for qrNode service\033[0;0m");
  
   
  var qrImagePath = Fs.resolvePath( storePath + fileName );
  var _args = {
    //"header": header,
    "imageFilename": qrImagePath //filenamePATH    
  }; 

  /*-----<Stores received image data>-----*/
  Fs.writeBinFileSync( qrImagePath, _qrImage );
  /*-----<Call QR ROS service through rosbridge>-----*/
  var returnMessage = ros.callService(qrRosService, _args);
  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  
  delete cache[randStr.toString()];
  /*--<Returned message from qr ROS service>--*/
  return returnMessage; 
}
