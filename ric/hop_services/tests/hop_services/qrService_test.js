/*!
 * @file qrService_test.js
 * @brief Test for qrNode service call.
 *
 */


var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");
var params = new ServerParams(false, "localhost", 9001, "", "");

var Fs = require(rapp_hop_path + "utilities/./fileUtils.js");
var HopUtils = require(rapp_hop_path + "utilities/./HopServiceUtils.js");


var qrImagePath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/qr_code_rapp.jpg";

//var qrData = Fs.readBinFileSync( qrImagePath ); 
//import service qrNode (_qrImage);

//var retMessage = qrNode(qrData).post(
  //function(data){
    //return data;
  //},
  //params 
//);

var hsu = new HopUtils();
hsu.init( params );
var retMessage = hsu.qrNode( qrImagePath );

console.log("\033[01;33mQR_Node service returned message:\033[01;36m");
console.log( retMessage );

