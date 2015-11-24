/*!
 * @file logger.js
 * @brief This module is used by the workerHandler to log messages
 *  regarding front-end HOP web services.
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


var Fs = require( '../fileUtils.js' );
var FileStreams = require( '../fileStreams.js' );
var sleep = require( '../sleep.js' );

// Default log directory path
var __logDir = '~/.hop/log/'

var __logDirCreated = false;
var __logFiles = [];
var __logFileStreams = {};

var color = {
  error:    '\033[1;31m',
  success:  '\033[1;32m',
  ok:       '\033[34m',
  yellow:   '\033[33m',
  clear:    '\033[0m'
}


/*!
 * @brief Creates directory where we store log files (.log).
 */
function createLogDir(logDir)
{
  if( !logDir )
  {
    console.log(color.error + '[Error]' + color.clear +
      ' - Failed to create directory {%s} for log files.', logDir);
    return false;
  }
  else
  {
    Fs.createDirRecur(logDir);
    __logDir = logDir;
    __logDirCreated = true;
  console.log('...logging to [%s]\r\n', __logDir);
  return true;
  }
}


/*!
 * @brief Instantiates log file of given worker
 */
function createLogFile( workerName )
{
  var logFilePath = __logDir + workerName + '.log';

  // Check if the log file was previously created.
  if ( logFileExists(logFilePath) )
  {
    console.log(color.error + '[Error]' + color.clear + ' - Log File {%s} already exists!!!');
    return undefined;
  }

  // Evaluate write methods on given log file.
  if ( Fs.write_file_sync(logFilePath, "") == true )
  {
    __logFiles.push(logFilePath);
    // Create and initiate a new write stream for this logfile.
    __logFileStreams[logFilePath] = new FileStreams.WriteStream(logFilePath);
    return logFilePath;
  }
  return undefined;
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

  console.log(color.error + '[Error]' + color.clear +
    '- Requested write to log file [%s] is not permitted' +
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


function registeredLogFiles(){
  return __logFiles;
}

// Module Exports
module.exports = {
  appendToLogFile: appendToLogFile,
  createLogFile: createLogFile,
  createLogDir: createLogDir,
  registeredLogFiles: registeredLogFiles
}
