/*!
 * @file init.js
 * @brief Initiates services found in services direcory on the same port.
 *  Each Service runs on a different worker.
 */

/**
 *  MIT License (MIT)
 *
 *  Copyright (c) <2014> <Rapp Project EU>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  Authors: Konstantinos Panayiotou
 *  Contact: klpanagi@gmail.com
 *
 */



// ------------------- GLOBALS ------------------- //
var __DEBUG__ = false;

var user = process.env.LOGNAME;


//  Hop services list by name
var srvList = [];
//  Hop services files found by full name
var srvFileList = [];


var Path = require('path');

//var module_path = Path.resolve(__dirname + '/../modules')
var module_path = __dirname + '/../modules/';
//console.log(module_path);

var Fs = require( module_path + 'fileUtils.js' );
var hop = require('hop');

// --------- Initiate Master Module ---------- //
var WorkerHandler = require( module_path + 'WorkerHandler/' + 'workerHandler.js' );
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
