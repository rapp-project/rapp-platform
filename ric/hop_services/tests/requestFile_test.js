/**
 * Simple file transfer test
 * Tested -> OK!
 */
var serverParams = require("../prototypes/./serverParams.js");
var rf = require("../utilities/requestFile/./requestFile.js");

var params = new serverParams(false, "localhost", 9001, "", "");
var file = new String();
file = "../tests/Robot_Blue.jpg";
destPath = "Robot_Blue_copy.jpg"
rf.requestFile(file, destPath, params);
