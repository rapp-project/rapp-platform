
var ROSbridge = require("../../utilities/./rosbridge.js");

var rosbridge = new ROSbridge();
rosbridge.connect();

var qrImage = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png" //filenamePATH
var serviceName = "/ric/face_detection_service";
var args = {
  "imageFilename": qrImage
};  

var retMsg = rosbridge.callServiceSync( serviceName, args );
console.log(retMsg);
rosbridge.close();
process.exit(0);
