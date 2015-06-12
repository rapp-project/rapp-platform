/*!
 * @file runWorkers.js
 * @brief Initiates services found in services/ direcory on the same port.
 *  Each Service runs on a different worker. They communicate.
 */


var user = process.env.LOGNAME;
var module_path = '../utilities/js/'

var Fs = require( module_path + 'fileUtils.js' );
var Path = require('path');

var fileList = Fs.ls_sync( __dirname );
var Services = [];

for (var i in fileList){
  //console.log( fileList[i] );
  if ( Path.extname( fileList[i] ) == '.js' ){
    console.log("Found js extension file: [%s]", fileList[i]);
    var regexp = /.service.js/g;
    if ( fileList[i].match(regexp) ){
      Services.push( fileList[i].replace( regexp, '' ) );
    } 
  }
}

console.log("\033[0;36mService js files found:\n\033[0;0m", Services);
        
var Workers = new Array();

Fs.rm_file_sync( __dirname + '/availableServices.txt' );

for (var i in Services){
  Workers.push( new Worker ( "./" + Services[i] + '.service.js' ) );
  console.log( "Created worker for service:" +
    "\033[0;32m[%s]\033[0;0m", Services[i] );

  /*-----<Store available uprunning services on a txt file>-----*/
  var regexp = /.service.js/g;
  var str = Services[i].toString().replace( regexp, '' );
  Fs.writeLine ( str,  __dirname + "/availableServices.txt" );  
};

