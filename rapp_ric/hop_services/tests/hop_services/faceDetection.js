/*!
 * @file faceDetection.js
 * @brief Test communication with faceDetection Hop-Service.
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
var faceImagePath = rapp_hop_path + "../test_auxiliary_files/Lenna.png";
// Set parameters used for accessing the hop server (Remote/Local).
var params = new ServerParams(false, "localhost", 9001, "", "");
var hsu = new HopUtils(); //create a hopUtils object
hsu.init( params, params ); //set local and remote parameters.
//Call faceDetection hop service
var retMessage = hsu.faceDetection( faceImagePath ); 
console.log("\033[01;33mFaceDetection service returned message:\033[01;36m");
console.log( JSON.parse(retMessage) ); //Output faceDetection results.
process.exit(0); //kill the current process (Exits).
