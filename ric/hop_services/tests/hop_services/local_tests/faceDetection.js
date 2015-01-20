/*!
 * @file qrService_test.js
 * @brief Test for faceDetection hop-service call.
 *
 */


var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");
var params = new ServerParams(false, "localhost", 9001, "", "");

var Fs = require(rapp_hop_path + "utilities/./fileUtils.js");
var HopUtils = require(rapp_hop_path + "utilities/./hopServices.js");


var faceImagePath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png";


var hsu = new HopUtils();
hsu.init( params, params );
var retMessage = hsu.faceDetection( faceImagePath );

console.log("\033[01;33mFaceDetection service returned message:\033[01;36m");
console.log( retMessage );
process.exit(0);
