/*!
 * @file runWorkers.js
 * @brief Initiates services found in services/ direcory on the same port.
 *  Each Service runs on a different worker. They communicate.
 */

var __DEBUG__ = true;

var user = process.env.LOGNAME;
//  Hop services list by name
var srvList = [];
//  Hop services files found by full name
var srvFileList = [];
//  Workers map
var hopSrvWorkers = {};

var module_path = '../utilities/js/'

var Fs = require( module_path + 'fileUtils.js' );
var Path = require('path');
var hop = require('hop');


Fs.rm_file_sync( __dirname + '/availableServices.txt' );

parse_services_dir();
register_slave_services();


service available_services()
{
  return srvList;
}

function register_slave_services()
{
  for (var i in srvFileList)
  {
    hopSrvWorkers[ srvList[i] ] = new Worker ( "./" + srvFileList[i] );

    console.log( "Created worker for service:" +
      "\033[0;32m[%s]\033[0;0m", srvList[i] );

    //  Register worker onmessage callback
    hopSrvWorkers[ srvList[i] ].onmessage = function(msg){
      if (__DEBUG__)
      {
        console.log("Received message from slave service:\033[0;32m %s\033[0m"
          , msg.data);
      }
    }
  }
};


function parse_services_dir()
{
    //Load files from this script directory
  var fileList = Fs.ls_sync( __dirname );

    //Load hop services form services directory
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
