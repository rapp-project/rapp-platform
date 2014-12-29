var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";
/*Require File Utilities module => make use of writeBinFileSync function*/
var Fs = require(rapp_hop_path + "utilities/./fileUtils.js");
var RosUtils = require(rapp_hop_path + "utilities/./RosUtils.js");

var qrRosService = "/ric/ros_nodes/qr_detection_service";

var ros = new RosUtils();
ros.init_bridge('');

service qrNode (_qrImage)
{
  var storePath = "~/hop_temps/"; 
  var fileName = "qrImage.jpg";
  var qrFoundMessage = false;

  console.log("\033[01;36mRequest for qrNode service\033[0;0m");
  
  var currentTime = new Date().getTime();
  var header = {
    "seq": 1,
    "stamp": currentTime,
    "frame_id": " "
  };
  var qrImagePath = Fs.resolvePath( storePath + fileName );
  var _args = {
    "header": header,
    "imageFilename": qrImagePath //filenamePATH    
  }; 

  Fs.writeBinFileSync( storePath + fileName, _qrImage );
  var returnMessage = ros.callService(qrRosService, _args);
  Fs.rmFileSync( storePath + fileName );

  return returnMessage; 
}
