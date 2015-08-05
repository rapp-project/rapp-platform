/*!
 * @file runWorkers.js
 * @brief Initiates services found in services/ direcory on the same port.
 *  Each Service runs on a different worker. They communicate.
 */

// ------------------- GLOBALS ------------------- //
var __DEBUG__ = true;

var user = process.env.LOGNAME;
//  Hop services list by name
var srvList = [];
//  Hop services files found by full name
var srvFileList = [];

var module_path = '../utilities/js/'

var Fs = require( module_path + 'fileUtils.js' );
var workerHandler = require( module_path + 'worker_handler.js' );
var Path = require('path');
var hop = require('hop');

var hostname = hop.hostname;
var port = hop.port;
// ---------------------------------------------- //


parse_services_dir();
workerHandler.register_workers(srvFileList, __dirname, srvList);

//workerHandler.kill_worker("qr_detection");

service available_services()
{
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
       sendResponse(srvList);
    }, this);
};



function parse_services_dir()
{
  //  Load files from this script directory
  var fileList = Fs.ls_sync( __dirname );

  //  Load hop services form services directory
  for (var i in fileList){
    if ( Path.extname( fileList[i] ) == '.js' ){
      var regexp = /.service.js/g;
      if ( fileList[i].match(regexp) ){
        console.log("Found hop service file: [%s]", fileList[i]);
        srvFileList.push( fileList[i] );
        srvList.push( fileList[i].replace( regexp, '') );
      }
    }
  }
};

