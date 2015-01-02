/*!
 * @file faceRemote_test.js
 * @brief Test REMOTE call to faceNode service
 *
 */

var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");

var Fs = require(rapp_hop_path + "utilities/./fileUtils.js");
var HopUtils = require(rapp_hop_path + "utilities/./HopServiceUtils.js");


var faceImagePath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna2.png";
var non_faceImagePath = rapp_hop_path + "tests/auxiliary/Robot_Blue.jpg"
var bigLenna = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png";


var params = new ServerParams(false, "155.207.19.37", 9001, "rappdev", "rappdev");

var hsu = new HopUtils();
hsu.init( params );
var retMessage = hsu.faceNode( faceImagePath );

console.log("\033[01;33mFace Node service returned message:\033[01;36m");
console.log( retMessage );


retMessage = hsu.faceNode( bigLenna );
console.log("\033[01;33mFace_Node service returned message:\033[01;36m");
console.log( retMessage );
