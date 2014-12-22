
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var RosUtils = require(rapp_hop_path + "utilities/./RosUtils_static.js");

RosUtils.init_bridge('');

var _service = "/ric/ros_nodes/qr_detection_service";
var currentTime = new Date().getTime();
var header = {
    "seq": 1,
    "stamp": currentTime,
    "frame_id": " "
  };


var _args = {
    "header": header,
    "imageFilename": "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/qr_code_rapp.jpg" //filenamePATH    
  };   
var returnMessage = RosUtils.callService(_service, _args);
console.log(returnMessage);

