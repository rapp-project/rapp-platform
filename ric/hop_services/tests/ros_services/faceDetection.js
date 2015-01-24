#!/usr/bin/env node


var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var rosbridge = require(rapp_hop_path + "utilities/./rosbridge.js");

var ros = new rosbridge();

var faceImage = "/home/klpanagi/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png" //filenamePATH

var serviceName = "/ric/face_detection_service";

var args = {
    "imageFilename": faceImage
};  


ros.callServiceAsync( serviceName, args )
//ros.close();
//process.exit(0);

function handle(data, flags) {
    console.log( data );
    this.close();
}


function sleep(milliseconds) {
  var start = new Date().getTime();
  for (var i = 0; i < 1e7; i++) {
    if ((new Date().getTime() - start) > milliseconds){
      break;
    }
  }
}


