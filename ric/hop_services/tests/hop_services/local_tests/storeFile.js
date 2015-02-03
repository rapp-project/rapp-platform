/*!
 * @file storeFile.js
 * @brief Test filetransfer methods from local to host.
 * Invoking a Hop client.
 */


/* Set the path to the rapp_workspace.
 * Required for loading required modules located in the rapp_workspace
 * directory
 */
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";
var rel_rappPath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/"

/*-------------------< Import required modules >------------------*/
var hopServices = require(rapp_hop_path + "utilities/./hopServices.js");
var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");
/*----------------------------------------------------------------*/

//Set system url of the file to be send to the hop service
var filePath = rel_rappPath + "test_auxiliary_files/Lenna.png";
//Set destination url. (Url to store the file).
var destPath = "~/Desktop/test"

// Set parameters used for accessing the hop server (Remote/Local).
var params = new ServerParams(false, "localhost", 9001, "", "");
var hop = new hopServices();
//hsu.init(params);
hop.sendFile( filePath, destPath, params );
process.exit(0); //kill the current process (Exits).
