var user = "klpanagi";
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";
var fileStream = require(rapp_hop_path + "utilities/./fileUtils.js");


/*###################--<Testing Section-->###################*/

/*<Test for resolvePath() function>*/
var resolvedPath = fileStream.resolvePath("~/Desktop");
console.log("\033[0;32mResolved Path: \033[01;31m[%s]",resolvedPath);

/*<Test for readSync function>*/
var filePath = rapp_hop_path + "tests/auxiliary/Robot_Blue.jpg" 
var file = fileStream.readFileSync(filePath);

/*<Test for writeSync function>*/
var destPath1 = "~/Documents/Robot_Blue_copy1.jpg";
fileStream.writeFileSync(destPath1, file);
var destPath2 = "~/Documents/Robot_Blue_copy2.jpg";
fileStream.writeFileSync(destPath2, file);

/*<Test for writeSync function>*/
var deleteFile = destPath2;
fileStream.rmFileSync(deleteFile);

