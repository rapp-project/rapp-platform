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

/**
 * @module
 *
 * @description NodeJs wrap methods while working with file system.
 *
 */

var fs = require('fs');
var path = require('path');


/**
 * Resolve a "might-be" relative path to system's absolute path.
 *
 * @param {String} _path Path to be resolved to absolute.
 * @returns {String} - absolute path.
 */
function resolvePath(_path) {
  var regexp = /~/g;
  var newPath = '';
  if (_path.match(regexp)) {
    var home = process.env.HOME;
    /*<Replaces "~" with "/home/user">*/
    newPath = _path.replace(regexp, home);
  }
  else {
    newPath = path.resolve(_path);
  }
  return newPath;
}


/**
 * Node.js readFileSync function wrapper
 *
 * @function readFileSync
 *
 * @param _file File to be read, specified by path.
 * @param _encoding Encoding type of returned data readen from the specified
 * file. Can be one of the following:
 *    "buffer" OR undefined Raw data from buffer.</li>
 *    "string/ascii" Ascii encoded string. </li>
 *    "string/binary" Binary encoded string. </li>
 *
 * @returns {Object} file - Custom file object.
 * @returns {String} file.data - Data payload.
 * @returns {String} file.encoding - Data encoding.
 * @returns {String} file.basename - File basename.
 * @returns {String} file.absolutePaht - File absolute system path.
 * @returns {Object} file.size - File size. Has the following parameters:
 *   <ul>
 *     <li> bytes </li>
 *     <li> kilobytes </li>
 *   </ul>
 */
function readFileSync(_fileUrl, _encoding) {
  var file = {
    data: undefined,
    encoding: undefined,
    basename: undefined,
    absolutePath: undefined,
    size: {
      bytes: undefined,
    }
  };
  var fileAbsPath = resolvePath(_fileUrl);
  if (fs.existsSync(fileAbsPath)) {
    file.absolutePath = fileAbsPath;
    file.basename = path.basename( fileAbsPath );
    var dataBuffer = fs.readFileSync( fileAbsPath );
    file.size.bytes = dataBuffer.length;
    encoding = _encoding || "none";
    switch (encoding) {
      case "buffer":
        file.data = dataBuffer;
        file.encoding = "raw";
        break;
      case "ascii":
        file.data = dataBuffer.toString( 'ascii' );
        file.encoding = "ascii";
        break;
      case "string/utf8":
        file.data = dataBuffer.toString( 'utf8' );
        file.encoding = "utf8";
        break;
      case "string/binary":
        var str = dataBuffer.toString( 'binary' );
        file.data = str;
        file.encoding = "binary";
        break;
      case "none":
        file.data = dataBuffer;
        file.encoding = "raw";
        break;
      default:
    }
    return file;
  }
  else {
    return 0;
  }
}


/**
 * Node.js writeFileSync method wrapper
 *
 * @function writeFileSync
 *
 * @param _dest Destination file name to write the data, specified by path.
 * @param _data Data to be written.
 *
 * @returns {boolean} - Success index on this operation.
 */
function writeFileSync(_destUrl, _data)
{
  var path =  resolvePath(_destUrl);

  try{
    fs.writeFileSync( path, _data );
  }
  catch(e){
    console.log(e);
    return false;
  }

  var filesize = fileSize(path);
  return true;
}


/**
 * Creates directory non-recursively
 *
 * @param {String} dirPath - Directory system path.
 *
 * @returns {boolean} - Success index on this operation.
 */
function createDir(dirPath) {
  var dir = resolvePath(dirPath);
  if (fs.existsSync(dir)) {
    return true;
  }

  try{
    fs.mkdirSync(dir);
  }
  catch(e){
    return false;
  }

  return true;
}


/**
 * Create directory recursively --> a/b/c/d
 *
 * @function createDirRecur
 *
 * @param {String} dirPath - Directory system path.
 */
function createDirRecur(dirPath) {
  dirPath = resolvePath(dirPath);
  if (fs.existsSync(dirPath)) {
    return true;
  }
  if (createDir(dirPath) === false) {
    // Create all the parents recursively
    createDirRecur(path.dirname(dirPath));

    // Then create the child directory
    createDirRecur(dirPath);
  }
  return true;
}


/**
 * @brief Remove local file given by system path. Synchronous.
 *
 * @function rmFileSync
 *
 * @param {String} filepath - The file system path
 */
function rmFileSync(filepath) {
  var _filepath =  resolvePath(filepath);
  if (fs.existsSync(_filepath) && isFile(_filepath)) {
    fs.unlinkSync(_filepath);
    return true;
  }
  else {
    return false;
  }
}


/**
 * @brief Remove local file given by system path. Asynchronous.
 *
 * @function rmFile
 *
 * @param {String} filepath - The file system path
 */
function rmFile(filepath) {
  var _filepath = resolvePath(filepath);
  fs.exists(_filepath, function(exists) {
    if (exists) {
      fs.unlink(_filepath, function(e) {
        if (e) {
          console.log(e);
        }
      });
    }
  });
}


/**
 * brief Reads the contents of a given directory path.
 *
 * @function lsSync
 *
 * @param {String} _dir - Directory path.
 * Works both with relative and absolute paths.
 *
 * @returns {Array} - Array that holds the contents of the directory.
 */
function lsSync(_dir) {
  var fileList = [];
  var dir = resolvePath(_dir);
  var files = fs.readdirSync(dir);

  for(var i in files) {
    var fullPath = dir + '/' + files[i];
    if (fs.statSync(fullPath).isDirectory()) {
      continue;
    }
    else {
      fileList.push(files[i]);
    }
  }
  return fileList;
}


/**
 * Writes ascii encoded strings in a give file.
 * DEPRECATED!!!
 *
 * @function text2File
 *
 * @param {String|Buffer}_data - Data to be written. Can be both a buffer or string.
 * @param {String} _filePath - Destination file path.
 * @return Undefined.
 */
function text2File (_data, _filePath) {
  var data = null;
  if (Buffer.isBuffer(_data)) {
    data = _data;
  }
  else if (typeof _data == 'string') {
    data = new Buffer( _data.length );
    data.write( _data );
  }
  else {
    return false;
  }

  var fd = fs.openSync(_filePath, 'w');
  var numBytes = fs.writeSync(fd, data, 0, data.length, null);
  fs.close(fd);
}


/**
 * Append a text into new line into destination file.
 *
 * @function appendLine
 *
 * @param {String} str - Text string to append to file.
 * @param {String} dest - Destination file path.
 *
 * @returns {boolean} - Success index on appendLine operation
 */
function appendLine(str, dest) {
  var destPath = resolvePath(dest);
  try {
    fs.appendFileSync(destPath, str + '\n');
    return true;
  }
  catch(e) {
    return false;
  }
}


/**
 * Getting File Size without Reading Entire File.
 *
 * @function fileSize
 *
 * @param {String} _fileURL File System Url.
 *
 * @return {number} - Size of the file in bytes.
 */
function fileSize(_filePath) {
  var _path =  resolvePath(_filePath);
  var stats = fs.statSync(_path);
  var filesize_bytes = stats.size;
  return filesize_bytes;
}


/**
 * Rename file. Can also be used as a funcitonality to copy files.
 *
 * @function renameFile
 *
 * @param fileOld Source file path.
 * @param fileNew Destination file path.
 *
 * @returns {boolean} - Success index of the rename-file operation.
 * True if file was succesfully renamed, false otherwise.
 */
function renameFile(file, dest) {
  var sourcePath = resolvePath(file);
  var destPath = resolvePath(dest);
  var destDir = parentDir(destPath);

  // If source file and destination file match then do not proceed.
  if (sourcePath == destPath) {
    return true;
  }

  // If parent directory of given destination file does not exist,
  // return false immediately.
  if (destDir === false || fs.existsSync(destDir) === false ||
    (! isFile(sourcePath)) ) {
      return false;
  }

  // Check if source file exists and destination directory also exists.
  if (fs.existsSync(sourcePath)) {
    try {
      fs.renameSync(sourcePath, destPath);
      //copyFile(sourcePath, destPath);
      //rmFile(sourcePath);
    }
    catch(e) {
      return false;
    }
    return true;
  }
  else {
    // If source file does not exist return false.
    return false;
  }
}


/**
 * Copies file from-To. Uses read/write streams and pipes.
 *
 * @function copyFile.
 *
 * @param file File (given with either relative or absolute path to copy
 * @param dest Destination to copy the file.
 *
 * @returns {boolean} - Success index of the copy-file operation.
 * True if file was succesfully copied, false otherwise.
 */
function copyFile(file, dest) {
  var sourcePath = resolvePath(file);
  var destPath = resolvePath(dest);
  var destDir = parentDir(destPath);

  // If source file and destination file match then do not proceed.
  if (sourcePath == destPath) {
    return true;
  }

  // If parent directory of given destination file does not exist,
  // return false immediately.
  if (destDir === false || fs.existsSync(destDir) === false) {
    return false;
  }

  // Check if source file exists and destination directory also exists.
  if (fs.existsSync(sourcePath)) {
    try {
      //fs.createReadStream(sourcePath).pipe(fs.createWriteStream(destPath));
      fs.writeFileSync(destPath, fs.readFileSync(sourcePath));
    }
    catch(e) {
      return false;
    }
    return true;
  }
  else {
    // If source file does not exist return false.
    return false;
  }
}


/**
 * Returns the parent directory name of a path.
 *
 * @function parentDir
 *
 * @param {String} _path - System path.
 *
 * @return {String} - The parent directory. In case of error a zero 0 value
 * will be returned.
 */
function parentDir(_path) {
  var absPath = resolvePath(_path);
  var _parentDir = '';
  try {
    _parentDir = path.dirname(absPath);
  }
  catch(e) {
    return false;
  }
  return _parentDir;
}


/**
 *  Check if a path is a directory
 *
 *  @function isDirectory
 *
 *  @param {String} _path - System path.
 *
 *  @returns {boolean} - True if is directory, false otherwise.
 */
function isDirectory(_path) {
  var dirPath = resolvePath(_path);
  var isDir = false;
  if (fs.existsSync(_path)) {
    isDir = fs.lstatSync(_path).isDirectory();
  }
  return isDir;
}


/**
 * Check if a path is a file.
 *
 * @param {String} _path - System path.
 * @returns {boolean} - True if is directory, false otherwise.
 */
function isFile(_path) {
  var filePath = resolvePath(_path);
  var isFile = false;
  if (fs.existsSync(_path)) {
    isFile = fs.lstatSync(_path).isFile();
  }
  return isFile;
}


function readTextFile(filepath, encoding) {
  // Encoding is set to utf8 by default
  encoding = encoding || 'utf8';

  if ( ! filepath ) {
    throw new Error("Not a filepath provided");
  }

  filepath = resolvePath(filepath);
  var text = '';
  try {
    text = fs.readFileSync(filepath, encoding);
  }
  catch(e) {
    throw new Error(e);
  }

  return text;
}

module.exports = {
  resolvePath: resolvePath,
  readFileSync: readFileSync,
  writeFileSync: writeFileSync,
  rmFile: rmFile,
  lsSync: lsSync,
  text2File: text2File,
  appendLine: appendLine,
  fileSize: fileSize,
  renameFile: renameFile,
  createDir: createDir,
  createDirRecur: createDirRecur,
  copyFile: copyFile,
  parentDir: parentDir,
  isDirectory: isDirectory,
  isFile: isFile,
  readTextFile: readTextFile
};
