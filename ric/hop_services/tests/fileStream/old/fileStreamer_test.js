var user = "klpanagi";
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";
var filePath = rapp_hop_path + "tests/auxiliary/Robot_Blue.jpg";
var destFilePath = rapp_hop_path + "tests/auxiliary/Robot_Blue_copy.jpg";
var destFilePath2 = "../../tests/auxiliary/Robot_Blue_copy2.jpg";

var fileStreamer = require(rapp_hop_path + "utilities/old/./fileStream.js");

var fs = new fileStreamer();

var data = fs.readSync(filePath);
fs.writeSync(destFilePath, data);
fs.writeSync(destFilePath2, data);

