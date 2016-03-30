/*!
 * @file service_handler.js
 * @brief Hop service handler.
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

var util = require('util');
var path = require('path');
var Fs = require(path.join(__dirname, 'fileUtils.js'));

var ON_ERROR_DEFAULT_MSG = "RAPP Platform Failure";


var cpInFile = function( filepath, destdir, unqId){
  if( ! filepath ){
    throw new Error("Not a source file path was provided");
  }
  if( ! destdir ){
    throw new Error("Not a destinatio directory path was provided");
  }
  // Accept copies even if not a unique id is provided. Simple Copy.
  unqId = unqId || '';

  var fileUrl = filepath.split('/');
  var filename = fileUrl[fileUrl.length -1];

  var cpFilepath = destdir + filename.split('.')[0] + '-'  +
    unqId + '.' + filename.split('.')[1];
  cpFilepath = Fs.resolvePath(cpFilepath);

  if (Fs.renameFile(filepath, cpFilepath) === false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    throw new Error(
      util.format("Failed to cp {%s} -> {%s}", filepath, cpFilepath)
      );
  }
  return cpFilepath;
};


/*!
 * @brief Sniff service arguments as obtained from the post request form.
 *  'json': {}  - Json string that contains the plain param=value
 *  'file': [] - An array that containes files posted in multipart/form-data
 */
var parseReq = function( svcInArgs, reqObj ){
  svcInArgs = svcInArgs || {};
  var jsonField = svcInArgs.json || '';
  var _file = svcInArgs.file || [];
  var obj = {};

  if ( (jsonField) &&  (isJson(jsonField)) ){
    obj = JSON.parse(jsonField);
  }
  obj.file = [];

  if ( _file ){
    if ( _file instanceof Array ){
       //An Array of files
      obj.file = svcInArgs.file;
    }
    else{
       //meaning single file.
      obj.file.push(_file);
    }
  }

  for ( var prop in svcInArgs ){
    if ( (prop !== "json") && (prop !== "file") ){
      obj[prop] = svcInArgs[prop];
    }
  }

  for( var i in reqObj ){
    reqObj[i] = (obj[i] !== undefined) ? obj[i] : reqObj[i];
  }
};


/*!
 *  Check if input value is in json string representation.
 *
 *  @returns True if isJson.
 */
var isJson = function( data ){
  try{
    JSON.parse(data);
  }
  catch(e){
    return false;
  }
  return true;
};



exports.ERROR_MSG_DEFAULT = ON_ERROR_DEFAULT_MSG;
exports.cpInFile = cpInFile;
exports.parseReq = parseReq;
