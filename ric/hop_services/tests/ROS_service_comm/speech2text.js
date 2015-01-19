
var ROSbridge = require("../../utilities/./rosbridge.js");

var rosbridge = new ROSbridge();
rosbridge.connect();

var audioFileUrl = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/test.flac" //filenamePATH
var serviceName = "/ric/speech_to_text_service";

var filename = {
  "data": audioFileUrl
};

var args = {};
args[ "filename" ] = filename; 

var retMsg = rosbridge.callServiceSync( serviceName, args );
console.log(retMsg);
rosbridge.close();
process.exit(0);
