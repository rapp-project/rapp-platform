
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var rosbridge = require( rapp_hop_path + "utilities/./rosbridge.js");

var ros = new rosbridge();
ros.connect();

var audioFileUrl = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/test.flac" //filenamePATH
var serviceName = "/ric/speech_to_text_service";

var filename = {
  "data": audioFileUrl
};

var args = {};
args[ "filename" ] = filename; 

var retMsg = ros.callServiceSync( serviceName, args );
console.log(retMsg);
ros.close();
process.exit(0);
