/*!
 * @file logger.js
 */

var Fs = require( '../fileUtils.js' );
var sleep = require( '../sleep.js' );

// Default log directory path
var __logDir = '~/.hop/log/'

var __logDirCreated = false;
var __logFiles = [];


/*!
 * @brief Sets the direcory where log files are stored
 */
function setLogDir(logDir)
{
  __logDir = logDir;
}


/*!
 * @brief Creates directory where we store log files (.log).
 */
function createLogDir(logDir)
{
  if(logDir == undefined || logDir == '')
  {
    Fs.createDirRecur(__logDir);
  }
  else
  {
    Fs.createDirRecur(logDir);
    setLogDir(logDir);
  }
  __logDirCreated = true;
  console.log('...logging to [%s]\r\n', __logDir);
  return;
}


/*!
 * @brief Instantiates log file of given worker
 */
function createLogFile( workerName )
{
  if( __logDirCreated == false )
  {
    console.log('Must create log directory first by calling the' +
      'relevant method [createLogDir]');
    return false;
  }

  var logFilePath = __logDir + workerName + '.log';

  if ( logFileExists(logFilePath) )
  {
    console.log('Log File already exists!!! Stopping creation');
    return false;
  }

  if ( Fs.write_file_sync(logFilePath, "") == true )
  {
    __logFiles.push(logFilePath);
    return true;
  }

  return false;
};


/*!
 * @brief Append to log file
 */
function appendToLogFile(workerName, logMsg)
{
  var logFilePath = __logDir + workerName + '.log';


  if ( logFileExists(logFilePath) )
  {
    /**
     * Force current thread to sleep. Fast i/o calls by the logger causes
     * an assertion in fs.c:
     *
     * hop: src/unix/fs.c:827: void uv__fs_done(struct uv__work *, int):
     * Assertion `(((const QUEUE *) (&(req->loop)->active_reqs) ==
     * (const QUEUE *) (*(QUEUE **) &((*(&(req->loop)->active_reqs))[0])))
     * == 0)' failed.
     *
     * @TODO Create and hold a writeStream for each logfile.
     *
     */
    sleep.sleepMS(150);
    Fs.appendLine(logMsg, logFilePath);
    return true;
  }

  console.log('Requested write to log file [%s] is not permitted' +
    'because log file was not previously created', logFilePath);
  return false;
};


/*!
 * @brief Checks if service given exists in registered services
 */
function logFileExists(logFile)
{
  if (__logFiles.indexOf(logFile) > -1) {return true;}
  else {return false;}
}


// Module Exports
module.exports = {
  appendToLogFile: appendToLogFile,
  createLogFile: createLogFile,
  createLogDir: createLogDir,
  setLogDir: setLogDir
}
