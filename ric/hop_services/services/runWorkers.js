/*!
 * @file runWorkers.js
 * @brief Initiates services found in services/ direcory on the same port.
 *  Each Service runs on a different worker. They communicate.
 */


var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + 
  "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";

var Fs = require( rapp_hop_path + "utilities/./fileUtils.js" );
var Path = require('path');

var fileList = Fs.getFilesListSync( rapp_hop_path + "services" );
var services = [];

for (var i in fileList){
  //console.log( fileList[i] );
  if ( Path.extname( fileList[i] ) == '.js' ){
    console.log("Found js extension file: [%s]", fileList[i]);
    var regexp = /.service/g;
    if ( fileList[i].match(regexp) ){
      services.push( fileList[i] );
    } 
  }
}

console.log("\033[0;36mService js files found:\n\033[0;0m", services);
        
var Workers = new Array();

for (var i in services){
  Workers.push( new Worker ( "./" + services[i] ) );
  console.log( "\033[01;35mCreated worker for service: [%s]", services[i] );
}
