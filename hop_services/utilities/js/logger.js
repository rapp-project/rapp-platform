/*!
 * @file logger.js
 */

var Fs = require( './fileUtils.js' );

var __logDir = '~/.hop/log/'

/*!
 * @brief Sets the direcory where log files are stored
 */
function setLogDir(logDir)
{
  __logDir = logDir;
}


/*!
 * @brief Instantiates log file of given worker
 */
function createLogFile(workerName)
{
  Fs.createDir(__logDir);
  var logFilePath = __logDir + workerName + '.log';
  Fs.write_file_sync(logFilePath, "");
};


/*!
 * @brief Append to log file
 */
function appendToLogFile(workerName, log)
{
  var logFilePath = __logDir + workerName + '.log';
  Fs.writeLine(log, logFilePath);
};


// Module Exports
module.exports = {
  appendToLogFile: appendToLogFile,
  createLogFile: createLogFile,
  setLogDir: setLogDir
}
