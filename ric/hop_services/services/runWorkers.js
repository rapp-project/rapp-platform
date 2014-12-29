/*!
 * @file runWorkers.js
 * @brief Initiates services found in services/ direcory on the same port.
 *  Each Service runs on a different worker. They communicate.
 */


var user = process.env.LOGNAME;
var rapp_hop_dir = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var Fs = require( rapp_hop_dir + 'utilities/./fileUtils.js');
var Path = require('path');

var fileList = Fs.getFilesListSync('../services');
var services = [];

for (var i in fileList){
  //console.log( fileList[i] );
  if ( Path.extname( fileList[i] ) == '.js' ){
    console.log("\033[0;32mFound js extension file: [%s]", fileList[i]);
    var regexp = /.service/g;
    if ( fileList[i].match(regexp) ){
      services.push( fileList[i] );
    } 
  }
}

console.log("Service js files found: ", services);
        
var Workers = new Array();

for (var i in services){
  Workers.push( new Worker ( "./" + services[i] ) );
  console.log( "\033[01;35mCreated worker for service: [%s]", services[i] );
}
