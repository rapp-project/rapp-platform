var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var RosUtils = require(rapp_hop_path + "utilities/./RosUtils.js");


var ros = new RosUtils();
ros.init_bridge('');


var faceImage = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png" //filenamePATH
var noFaceImage = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/qr_code_rapp.jpg" //filenamePATH

var serviceName = "/ric/face_detection_service";
var currentTime = new Date().getTime();

var args = {
    //"header": header,
    "imageFilename": faceImage
};  


ros.callService( serviceName, args );
var msg = ros.getResponseMsg();
console.log(msg);

args["imageFilename"] = noFaceImage;
ros.init_bridge('');
ros.callService( serviceName, args );
console.log(msg);
