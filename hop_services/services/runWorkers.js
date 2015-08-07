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
var Path = require('path');
var hop = require('hop');

// --------- Initiate Master Module ---------- //
var master = new ( require( module_path + 'master.js' ) ).Master();
// ------------------------------------------- //

var hostname = hop.hostname;
var port = hop.port;
// ---------------------------------------------- //

parse_services_dir(__dirname);

/* ---------< Register services to master >----------- */
master.registerWorkers(srvFileList, __dirname, srvList);
/* --------------------------------------------------- */

function parse_services_dir(dir)
{
  //  Load files from this script directory
  var fileList = Fs.ls_sync( dir );

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


//service available_services()
//{
  //return hop.HTTPResponseAsync(
    //function( sendResponse ) {
       //sendResponse(srvList);
    //}, this);
//};




