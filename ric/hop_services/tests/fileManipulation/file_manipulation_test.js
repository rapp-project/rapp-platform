var user = "klpanagi" 
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";
var filePath = rapp_hop_path + "tests/auxiliary/Robot_Blue.jpg";
var destFilePath = rapp_hop_path + "tests/auxiliary/Robot_Blue_copy.jpg"
var destFilePath2 = rapp_hop_path + "tests/auxiliary/Robot_Blue_copy2.jpg"

var fread = require(rapp_hop_path + "utilities/./readFile.js");
var fwrite = require(rapp_hop_path + "utilities/./writeFile.js");
var frm = require(rapp_hop_path + "utilities/./rmFile.js");


var data = fread.readFile(filePath);
fwrite.writeFile(destFilePath, data);
fwrite.writeFile(destFilePath2, data);
var eval = frm.rmFile(destFilePath);
console.log("\033[01;31m Remove file returned [%s]", eval);


