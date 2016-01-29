/*!
 * @file workerHandler.js
 * @brief Worker handler module is used by the main process to handle with
 *  hop service workers. Bipolar communication with all the workers.
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


/**
 * Global Module variables are assigned with one underscore "_xx" as an
 * extenstion
 *
 * Global variables are assigned with two underscores "__xx" infront
 * of the variable decleration.
 *
 * Loaded module variables declerations start with Capital letters.
 */

__POP_LOG_WINDOWS = false;

var path = require('path');
var hop = require('hop');

const ENV = require( path.join(__dirname, '../..', 'env.js') );
const PKG_DIR = ENV.PATHS.PKG_DIR;
const INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var Rsg_ = require ( path.join(INCLUDE_DIR, 'common', 'randStringGen.js') );
var RunTime_ = require( './runtime.js' );
var Logger_ = require( './logger.js' );
var Cache_ = require( './cache.js' );
var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );
var exec = require('child_process').exec;

/* -------------------- Cache directories ------------------------ */
const SERVICES_CACHE_DIR = ENV.PATHS.SERVICES_CACHE_DIR;
const __serverCacheDir = ENV.PATHS.SERVER_CACHE_DIR;
/* --------------------------------------------------------------- */

/* ----------------< Logging configuration >---------------- */
const __logDirBase = ENV.PATHS.LOG_DIR;
const __logDir = __logDirBase + RunTime_.getDate() + '/';
Logger_.createLogDir(__logDir);  // Create log directory if it does not exist.
/* --------------------------------------------------------- */

/* --------< Initiate Random String Generator Module >------- */
const __randStrLength = 5;
var RandStrGen_ = new Rsg_( __randStrLength );
/* ---------------------------------------------------------- */

/* ----- < Workers info hold >------ */
var __workers = {};
var __services = [];
var __workerId = {};
/* --------------------------------- */

/* -----------< File cache configuration >--------------- */
Cache_.createCacheDir(SERVICES_CACHE_DIR);
Cache_.createCacheDir(__serverCacheDir);
/* ------------------------------------------------------ */

var __child_processes = {};

var color = {
  success:  '\033[1;32m',
  error:    '\033[1;31m',
  ok:       '\033[1;34m',
  yellow:   '\033[33m',
  clear:    '\033[0m',
  cyan:     '\033[36m'
};


function popLogWindow(logFile){
  var cmd = 'xterm -e tail -f ' + logFile;
  __child_processes[logFile] = exec(cmd, function(error, stdout, stderr){
    if(error !== null){
      console.log('child process exec error: ' + error);
    }
  });
}

/**
 * @function registerToLogger
 *
 * Registers given worker to logger.
 */
function registerToLogger(workerName)
{
  logFilePath = Logger_.createLogFile(workerName);
  if(logFilePath !== undefined && __POP_LOG_WINDOWS){
    popLogWindow(logFilePath);
  }
}


/*!
 * @brief Used to handle WorkerToMaster messages.
 */
function handleMsg(msg)
{
  var srvId = msg.id;
  var srvName = msg.name;
  var msgId = msg.msgId;
  var data = msg.data;

  var timeNow = RunTime_.getTime();

  switch (msgId)
  {
    case 'log':
      var logMsg = '[' + timeNow + ']: ' + data;
      Logger_.appendToLogFile(srvName, logMsg);
      break;
    default:
      break;
  }
}


/*!
 * @brief Creates a unique id for worker and call worker to store it.
 */
function set_worker_id(workerName)
{
  var unqId = RandStrGen_.createUnique();

  var msg = {
    cmdId: 2055,
    data: unqId
  };

  // ------- <Send workerId> -------- //
  __workers[ workerName ].postMessage(msg);
  __workerId[ workerName ] = unqId;
}



/*!
 * @brief Kill worker given by Name
 */
function kill_worker(workerName)
{
  if (service_exists(workerName))
  {
    __workers[workerName].terminate();
    console.log("Terminated worker: %s", workerName);
  }
}


/*!
 * @brief Register web services as workers.
 * @param worker [{file: <absolute path to worker file>, name: <workerName>}]
 *  (Object literal Array)
 */
function register_worker(worker)
{
  try{
  __workers[ worker.name ] = new Worker ( worker.file );
  }
  catch(e){
    console.log(color.error + '[Error] - Failed to Launch Web Service:\n' +
      'file:[%s], name:[%s]' + color.clear,
      worker.file, worker.name);
    console.log('- Trace:\n' + e.toString());
    return false;
  }

  __services = __services.concat(worker.name);

  set_worker_id( worker.name );
  registerToLogger( worker.name );

  var serviceUrl = service_url( worker.name );

  console.log(color.success + '[OK]' + color.clear +
    ' - Initiated HOP Web Service: \n' + color.cyan +
    serviceUrl + color.clear);

  //  Register 'this' worker onmessage callback
  __workers[ worker.name ].onmessage = function(msg){
    handleMsg(msg.data);
  };
}


/*!
 * @brief Checks if service given exists in registered services
 */
function service_exists(srvName)
{
  if (__services.indexOf(srvName) > -1) {return true;}
  else {return false;}
}


/*!
 * @brief Terminates WorkerHandler. Sends termination signal to workers
 * and finalize logger
 */
function terminate()
{
  for(var i in __services)
  {
    var logMsg = 'Termination signal. Exiting...';
    Logger_.appendToLogFile(__services[i], logMsg);
  }
}


function service_url(srvName){
  return 'http://' + hop.hostname + ':' + hop.port + '/hop/' + srvName;
}


// Export Module
module.exports = {
  register_worker: register_worker,
  terminate: terminate
};
