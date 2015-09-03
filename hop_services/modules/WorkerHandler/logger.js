/*!
 * @file logger.js
 */

var Fs = require( '../fileUtils.js' );
var FileStreams = require( '../fileStreams.js' );
var sleep = require( '../sleep.js' );

// Default log directory path
var __logDir = '~/.hop/log/'

var __logDirCreated = false;
var __logFiles = [];
var __logFileStreams = {};


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
  // Check if there the log directory was previously created.
  if( __logDirCreated == false )
  {
    console.log('Must create log directory first by calling the' +
      'relevant method [createLogDir]');
    return false;
  }

  var logFilePath = __logDir + workerName + '.log';

  // Check if the log file was previously created.
  if ( logFileExists(logFilePath) )
  {
    console.log('Log File already exists!!! Stopping creation');
    return false;
  }

  // Evaluate write methods on given log file.
  if ( Fs.write_file_sync(logFilePath, "") == true )
  {
    __logFiles.push(logFilePath);
    // Create and initiate a new write stream for this logfile.
    __logFileStreams[logFilePath] = new FileStreams.WriteStream(logFilePath);
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
    __logFileStreams[logFilePath].write(logMsg + '\n');
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
