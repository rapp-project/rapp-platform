/*!
 * @file speech2text.js
 * @brief Test communication with speech2text Hop Service.
 *  Invoking a Hop Client.
 */


/* Set the path to the rapp_workspace.
 * Required for loading required modules located in the rapp_workspace
 * directory
 */
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/rapp_ric/hop_services/";

/*-------------------< Import required modules >------------------*/
var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");
var hopServices = require(rapp_hop_path + "utilities/./hopServices.js");
/*----------------------------------------------------------------*/

// Set audio file system url
var audioFileUrl = rapp_hop_path + "../test_auxiliary_files/test.flac";
// Set parameters used for accessing the hop server (Remote/Local).
var params = new ServerParams(false, "localhost", 9001, "", "");
var hop = new hopServices();
hop.init( params, params );

var ontologyQuery = "Oven"
//Call faceDetection hop service
var retMessage = hop.ontology_subclassesOf( ontologyQuery );
console.log("\033[01;33mOntology SubclassOf service returned message:\033[01;36m");
console.log( JSON.parse(retMessage) );
process.exit(0); //kill the current process (Exits).
