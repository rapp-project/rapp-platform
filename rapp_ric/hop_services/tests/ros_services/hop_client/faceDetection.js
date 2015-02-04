
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var rosbridge = require(rapp_hop_path + "utilities/./rosbridge.js");

var ros = new rosbridge();

var faceImage = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png" //filenamePATH

var serviceName = "/ric/face_detection_service";

var args = {
    "imageFilename": faceImage
};  

ros.connect(); //connects to rosbridge
// Call speech2text ROS service and return message from rosbridge
var msg = ros.callServiceSync( serviceName, args ); 
console.log( msg );
ros.close(); // Closes connection to rosbridge.
process.exit(0) // Kills the current process and exits.
