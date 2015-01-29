/*!
 * @file storeFile_test.js
 * @brief Test filetransfer methods from local to host.
 */

var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";
var rel_rappPath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/"
var filePath = rel_rappPath + "test_auxiliary_files/Lenna.png";
var destPath = "~/Desktop/test"

var hopServices = require(rapp_hop_path + "utilities/./hopServices.js");
var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");


var params = new ServerParams(false, "localhost", 9001, "", "");
var hop = new hopServices();
//hsu.init(params);
hop.sendFile( filePath, destPath, params );
process.exit(0);
