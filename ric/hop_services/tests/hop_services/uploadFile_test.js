/*!
 * @file uploadFile_test.js
 *
 * Simple file transfer request test. Uses a hop service in order to 
 * transmit the requested file [from-to].
 *
 * On the client-side, where this test is executed, a serveFile hop service
 * must exist and be up and running.
 */

var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + 
  "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var HopUtils = require(rapp_hop_path + "utilities/./HopServiceUtils.js");
var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");

var file = rapp_hop_path + "../test_auxiliary_files/Lenna.png";
var destPath = "~/hop_temps/test.png";
var startT, endT;

var remoteServerParams = new ServerParams(
  false, "155.207.19.37", 9001, "rappdev", "rappdev");
var localServerParams = new ServerParams( false, "155.207.33.185", 9001, "", "" );

var hsu = new HopUtils();
hsu.init( localServerParams, remoteServerParams );

startT = new Date();
hsu.uploadFile( file, destPath, localServerParams, remoteServerParams );
endT = new Date();

console.log('Uploading file Operation took ' +
  (endT.getTime() - startT.getTime()) + ' msec');






