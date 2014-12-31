
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var RosUtils = require(rapp_hop_path + "utilities/./RosUtils.js");


var ros = new RosUtils();
ros.init_bridge('');


var qrImage = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/qr_code_rapp.jpg" //filenamePATH
var serviceName = "/ric/ros_nodes/qr_detection_service";
var currentTime = new Date().getTime();

var args = {
    //"header": header,
    "imageFilename": qrImage
};  


ros.callService( serviceName, args );
var msg = ros.getResponseMsg();

console.log(msg);

//ros.closeSocket();
ros = null;
delete ros;

