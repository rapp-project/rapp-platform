/*!
 * @file qrService.js
 * @brief Test communication with qrDetection Hop-Service.
 *  Invoking a Hop client.
 */


/* Set the path to the rapp_workspace.
 * Required for loading required modules located in the rapp_workspace
 * directory
 */
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user +
  "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/rapp_ric/hop_services/";

/*-------------------< Import required modules >------------------*/
var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");
var HopUtils = require(rapp_hop_path + "utilities/./hopServices.js");
/*----------------------------------------------------------------*/

// Set image file system url
var qrImagePath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/rapp_ric/test_auxiliary_files/qr_code_rapp.jpg";
// Set parameters used for accessing the hop server (Remote/Local).
var params = new ServerParams(false, "localhost", 9001, "", "");
var hsu = new HopUtils();
hsu.init( params );
//Call faceDetection hop service
var retMessage = hsu.qrDetection( qrImagePath, params );
console.log("\033[01;33mQR_Node service returned message:\033[01;36m");
console.log( JSON.parse(retMessage) );
process.exit(0); //Kill the current process (Exits).
