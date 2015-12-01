/***
 * Copyright 2015 RAPP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Konstantinos Panayiotou
 * Contact: klpanagi@gmail.com
 *
 */


/***
 * @fileOverview
 *
 *  Parse Services directory. Files with a:
 *    .service.js
 *  extension are loaded and registered, as HOP Web services,
 *  under the HOP server.
 *
 *  This source file is loaded by the HOP server on execution-time.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var __DEBUG__ = false;

var hop = require('hop');
var path = require('path');

var hostname = hop.hostname;
var port = hop.port;

var __includeDir = path.join(__dirname, '..', 'modules');

var Fs = require( path.join(__includeDir, 'common', 'fileUtils.js') );

var WorkerHandler = require( path.join(__includeDir, 'WorkerHandler',
    'workerHandler.js') );

var color = {
  error:    String.fromCharCode(0x1B) + '[1;31m',
  success:  String.fromCharCode(0x1B) + '[1;32m',
  ok:       String.fromCharCode(0x1B) + '[34m',
  yellow:   String.fromCharCode(0x1B) + '[33m',
  clear:    String.fromCharCode(0x1B) + '[0m'
};

//  Hop services list by name
var srvList = [];
//  Hop services files found by full name
var srvFileList = [];


/***
 *  Parse directories where services are stored. Javascript files with a:
 *    .service.js
 *  extension are imported and registered as HOP Web Services,
 *  under the HOP Server.
 *
 */
parse_services_dir(__dirname);


function parse_services_dir(dir)
{
  // Load files from located under given as input directory (dir)
  var workerFileList = Fs.lsSync( dir );

  // Loop through files list and search for worker services declered files
  for (var i in workerFileList){
    if ( path.extname( workerFileList[i] ) == '.js' ){
      var regexp = /.service.js/g;
      // If match then it is a hop service file.
      if ( workerFileList[i].match(regexp) ){
        console.log("Found hop service file: [%s]", workerFileList[i]);
        /* -----------< Register worker >--------- */
        var worker = {
          file: dir + '/' + workerFileList[i],
          name: workerFileList[i].replace( regexp, '' )
        };
        WorkerHandler.registerWorker(worker);
        /* --------------------------------------- */
      }
    }
  }
}


/* ------------------- Process Handling ----------------- */

// Prevents the program from closing instantly
// Does not work with hop-client!!!!!
process.stdin.resume();

process.on('SIGTERM',
  function(){
    console.log('Exiting with code: ');
    WorkerHandler.terminate();
  }
);


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
