/*!
 * @file requestFile_local.js
 *
 * Simple file transfer request test. Uses a hop service in order to 
 * transmit the requested file [from-to].
 */

var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var HopUtils = require(rapp_hop_path + "utilities/./HopServiceUtils.js");
var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");

var file = rapp_hop_path + "../test_auxiliary_files/Lenna.png";
var destPath = "~/hop_temps/test.png";
var startT, endT;

var remoteServerParams = new ServerParams( false, "155.207.33.185", 9001, "", "" );
var localServerParams = new ServerParams( false, "155.207.33.185", 9001, "", "" );

var hsu = new HopUtils();
hsu.init( localServerParams, remoteServerParams );

startT = new Date();
hsu.getFile( file, destPath );
endT = new Date();

console.log('Transfering and storing binary file Operation took ' +
  (endT.getTime() - startT.getTime()) + ' msec');






