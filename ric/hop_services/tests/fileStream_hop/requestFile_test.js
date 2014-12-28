/**
 * Simple file transfer test. Uses a hop service in order to 
 * transmit the requested file [from-to].
 * Tested -> OK!
 */
var user = "klpanagi" 
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var serverParams = require(rapp_hop_path + "prototypes/./serverParams.js");
var rf = require(rapp_hop_path + "utilities/./requestFile.js");
var fsbin = require(rapp_hop_path + "utilities/./fsbinary.js");

var params = new serverParams(false, "localhost", 9001, "", "");

file = rapp_hop_path + "tests/auxiliary/Robot_Blue.jpg";
destPath = rapp_hop_path + "tests/auxiliary/Robot_Blue_copy.jpg";

var data1 = rf.requestFile(file, true, params);

fsbin.writeFileSync(destPath, data1);


