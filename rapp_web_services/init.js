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
 *  This source file is loaded by the HOP server on execution-time.
 *  Instatiates a ServiceHandler and registers found workers.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var hop = require('hop');
var path = require('path');

const ENV = require( path.join(__dirname, 'env.js') );
const PKG_DIR = ENV.PATHS.PKG_DIR;
const INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;
const SERVICES_CACHE_DIR = ENV.PATHS.SERVICES_CACHE_DIR;
const SERVER_CACHE_DIR = ENV.PATHS.SERVER_CACHE_DIR;
/* --------------------------------------------------------------- */

var hostname = hop.hostname;
var port = hop.port;
var workers = require('./config/services/workers.json');

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );
var logger = new (require( path.join(INCLUDE_DIR, 'common', 'logger.js') ))(
  {console: true, file: false, ns: 'Service Handler'});
if( createCacheDir(SERVICES_CACHE_DIR) ){
  logger.info('...Services caching in ' + SERVICES_CACHE_DIR);
}else{
  logger.error('Failed to create cache directories: ' + SERVICES_CACHE_DIR);
}

var srvHandler = new ( require( path.join(INCLUDE_DIR, 'servicehandler',
    'service_handler.js') ))({logger: logger});


launchWorkers(workers);


/** ---------------------- [Util functions] --------------------- */
/** ------------------------------------------------------------- */

function createCacheDir( dir ){
  if( dir === undefined || dir === '' )
  {
    return false;
  }
  if( ! Fs.createDirRecur(dir) ){
    return false;
  }
  return true;
}


function launchWorkers( workers ){
  for(var w in workers){
    if( (workers[w].launch === true) || ( workers[w].launch) === undefined ){
      var worker = {
        file: path.join(__dirname, workers[w].path),
        name: w,
      };
      srvHandler.registerWorker( worker );
    }
  }
}

/** ------------------------------------------------------------- */

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
