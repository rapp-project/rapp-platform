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
 * @description NodeJs wrap methods while working with file system.
 *
 * @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 */


var colors = {
  success: '\033[0;32m',
  error: '\033[0;31m',
  clear: '\033[0m',
  ok: '\033[0;34m'
};


var fs = require( 'fs' );
var Fs = require( './fileUtils.js' );


/**
 * Writable file streams.
 *
 * @class WriteStream
 * @param {String} filePath The file-path to open the writable stream
 *
 * @property wstream_ - Writable Stream.
 * @property {String} endPoint_ - Stream end-point.
 * @property {Method} close(_callback) - Close the WriteStream. Has an optional
 * callback parameter to be called when an error has occured while closing
 * the writable stream
 * @property {Method} write(data) - Append data to the write-stream.
 *
 */
function WriteStream( filePath )
{
  /** @lends WriteStream.prototype */
  this.wstream_ = undefined;
  this.endPoint_ = filePath || '';

  if( filePath )  { this.create(filePath); }

}


/**
 * @description Close the writable stream
 */
WriteStream.prototype.close = function( _callack )
{
  try { if(this.wstream_) { this.wstream_.end(); } }
  catch(e)
  {
    var errorMsg = 'Thrown an exception on line 64' +
        ' [fileStreams.js] while trying to close stream';
      console.log(colors.error + errorMsg + colors.clear);
    if(_callback) { _callback(); }
    else { return false; }
  }

};


/**
 * @description Create a writable stream
 *
 */
WriteStream.prototype.create = function( streamEndPoint )
{
  var absFilePath = Fs.resolvePath( streamEndPoint );
  try{
    this.wstream_ = fs.createWriteStream( absFilePath );
  }
  catch(e){
    console.log(colors.error + e.toString() + colors.clear);
    this.wstream_ = undefined;
    return false;
  }

  console.log(colors.ok + 'Initiated Writable Stream '+ colors.clear +
    "[%s]", absFilePath);
  return true;
};


/**
 * @description Flush data into the writable stream
 */
WriteStream.prototype.write = function( data ){
  this.wstream_.write( data );
};



module.exports = {
  WriteStream: WriteStream
};
