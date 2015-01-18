/*!
 * @file requestFileFromService_test.js
 *
 * Simple file transfer test. Uses a hop service in order to 
 * transmit the requested file [from-to].
 * Tested -> OK!
 */

var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";
var rel_rappPath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/"

var HopUtils = require(rapp_hop_path + "utilities/./HopServiceUtils.js");
var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");

var file = rel_rappPath + "tests/auxiliary/Robot_Blue.jpg";
var destPath = "~/Documents/test.jpg";

//var serverParams = {
  //async: false,
  //host: "localhost",
  //port: 9001,
  //user: "",
  //passwd: ""
//};

var params = new ServerParams(false, "localhost", 9001, "", "");
var hsu = new HopUtils();
hsu.init(params);
hsu.getFile(file, destPath);





