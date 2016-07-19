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
 * @file
 * @description NodeJs wrap methods while working with file streams.
 *
 * @author Konstantinos Panayiotou <klpanagi@gmail.com>
 * @copyright Rapp Project EU 2015
 */


var colors = {
  success: '\033[0;32m',
  error: '\033[0;31m',
  clear: '\033[0m',
  ok: '\033[0;34m'
};


var fs = require('fs');
var Fs = require('./fileUtils.js');


/**
 * Writable file streams.
 *
 * @class WriteStream
 * @param {String} filePath The file-path to open the writable stream
 *
 */
function WriteStream(filePath) {
  /** @lends WriteStream.prototype */
  this.wstream_ = undefined;
  this.endPoint_ = filePath || '';

  if (filePath)  {
    this.create(filePath);
  }

}


/**
 * @description Close the writable stream
 *
 * @function close
 * @memberOf WriteStream
 */
WriteStream.prototype.close = function(_callack) {
  try {
    if (this.wstream_) {
      this.wstream_.end();
    }
  }
  catch(e) {
    console.log(e);
    if (_callback) {
      _callback();
    } else {
      return false;
    }
  }

};


/**
 * @description Create a writable stream
 *
 * @function close
 *
 * @param {string} streamEndPoint - The stream end-point
 * @memberOf WriteStream
 */
WriteStream.prototype.create = function(streamEndPoint) {
  var absFilePath = Fs.resolvePath(streamEndPoint);
  try{
    this.wstream_ = fs.createWriteStream(absFilePath);
  }
  catch(e){
    console.log(e);
    this.wstream_ = undefined;
    return false;
  }

  console.log('Initiated Writable Stream [%s]', absFilePath);
  return true;
};


/**
 * @description Flush data into the writable stream
 *
 * @function write
 * @memberOf WriteStream
 *
 * @param data - Data to flush to the stream
 */
WriteStream.prototype.write = function(data) {
  this.wstream_.write(data);
};



module.exports = {
  WriteStream: WriteStream
};
