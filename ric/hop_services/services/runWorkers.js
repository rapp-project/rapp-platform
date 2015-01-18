/*!
 * @file runWorkers.js
 * @brief Initiates services found in services/ direcory on the same port.
 *  Each Service runs on a different worker. They communicate.
 */


var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + 
  "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";

var Fs = require( /*rapp_hop_path +*/ "../utilities/./fileUtils.js" );
//var fs = require( 'fs' );
var Path = require('path');

var fileList = Fs.getFilesListSync( /*rapp_hop_path +*/ "../services" );
var Services = [];

for (var i in fileList){
  //console.log( fileList[i] );
  if ( Path.extname( fileList[i] ) == '.js' ){
    console.log("Found js extension file: [%s]", fileList[i]);
    var regexp = /.service/g;
    if ( fileList[i].match(regexp) ){
      Services.push( fileList[i] );
    } 
  }
}

console.log("\033[0;36mService js files found:\n\033[0;0m", Services);
        
var Workers = new Array();

Fs.rmFileSync( 'availableServices.txt' );

for (var i in Services){
  Workers.push( new Worker ( "./" + Services[i] ) );
  console.log( "\033[01;35mCreated worker for service: [%s]\033[0;0m", Services[i] );

  /*-----<Store available uprunning services on a txt file>-----*/
  var regexp = /.service.js/g;
  var str = Services[i].toString().replace( regexp, '' );
  Fs.writeLine ( str,  "availableServices.txt" );  
};

