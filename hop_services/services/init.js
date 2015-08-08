/*!
 * @file runWorkers.js
 * @brief Initiates services found in services direcory on the same port.
 *  Each Service runs on a different worker.
 */

// ------------------- GLOBALS ------------------- //
var __DEBUG__ = false;

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
//var Master = new ( require( module_path + 'master.js' ) ).Master();
var WorkerHandler = require( module_path + 'workerHandler.js' );
// ------------------------------------------- //

var hostname = hop.hostname;
var port = hop.port;
// ---------------------------------------------- //

parse_services_dir(__dirname);


function parse_services_dir(dir)
{
  // Load files from located under given as input directory (dir)
  var workerFileList = Fs.ls_sync( dir );

  // Loop through files list and search for worker services declered files
  for (var i in workerFileList){
    if ( Path.extname( workerFileList[i] ) == '.js' ){
      var regexp = /.service.js/g;
      // If match then it is a hop service file.
      if ( workerFileList[i].match(regexp) ){
        console.log("Found hop service file: [%s]", workerFileList[i]);
        /* -----------< Register worker >--------- */
        var worker = {
          file: dir + '/' + workerFileList[i],
          name: workerFileList[i].replace( regexp, '' )
        }
        WorkerHandler.registerWorker(worker);
        /* --------------------------------------- */
      }
    }
  }
};


/* ------------------- Process Handling ----------------- */

// Prevents the program from closing instantly
// Does not work with hop-client!!!!!
process.stdin.resume();

process.on('SIGTERM',
  function(){
  console.log('Exiting with code: ');
  WorkerHandler.terminate();
  }
  )


//process.on('exit',
  //function(){
  //console.log('Exiting with code: ');
  //WorkerHandler.terminate();
  //}
  //)


//process.on('SIGINT',
  //function(){
  //console.log('Exiting with code: ');
  //WorkerHandler.terminate();
  //}
  //)


//process.on('SIGKILL',
  //function(){
  //console.log('Exiting with code: ');
  //WorkerHandler.terminate();
  //}
  //)

/* ---------------------------------------------------- */
