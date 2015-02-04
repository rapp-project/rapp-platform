var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");

var Fs = require(rapp_hop_path + "utilities/./fileUtils.js");
var HopUtils = require(rapp_hop_path + "utilities/./HopServiceUtils.js");

var startT, endT;

//var faceImagePath = rapp_hop_path + "../test_auxiliary_files/Lenna.png";
var faceImagePath = '~/Pictures/face1.jpg'

var remoteParams = new ServerParams(false, "155.207.19.37", 9001, "rappdev", "rappdev");
var localParams = new ServerParams(false, "155.207.33.185", 9001, "klpanagi", "peace");

var hsu = new HopUtils();

startT = new Date();
var retMessage = hsu.face_byPath( faceImagePath, localParams, remoteParams );
endT = new Date();
console.log( '\033[01;35m' );
console.log(retMessage.values.faces_up_left);
console.log( '\033[0;0m' );
console.log( '\033[01;36m' );
console.log(retMessage.values.faces_down_right);
console.log( '\033[0;0m' );


console.log('Face detection Operation took ' +
  (endT.getTime() - startT.getTime()) + ' msec');


