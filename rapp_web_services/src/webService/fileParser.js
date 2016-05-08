/***
 * Copyright 2016 RAPP
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
 *
 *  Parse request object for transfered files and append in req.files
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2016
 */

var fs = require('fs');


function FileParser() {
}


FileParser.prototype.isFile = function(path){
  if(! (typeof path === 'string' || path instanceof String) ){
    return false;
  }
  var _isfile = false;
  if( fs.existsSync(path) ) {_isfile = fs.lstatSync(path).isFile();}
  return _isfile;
}


FileParser.prototype.isArrayOfFiles = function(array){
  if(array instanceof Array && array.length){
    for(var i in array){
      if(! this.isFile(array[i])){
        return false
      }
    }
    return true;
  }
  return false;
}


/*!
 * @brief Parses the request arguments for files. Adds any received files
 * in req.files. For example "imagefile" can be retrieved from:
 * req.files.imagefile[0]
 * It is an array because multiple files can be received from
 * multipart/form-data requests as an Array of files, having the same form name.
 *
 * @param {Object} req - The request object.
 */
FileParser.prototype.call = function(req) {
  var _files = {};
  var removeFromBody = [];

  for (var i in req.body){
    if (this.isArrayOfFiles(req.body[i])) {
      _files[i] = req.body[i];
      delete req.body[i];
    }
    else if(this.isFile(req.body[i])){
      // Meaning single file.
      _files[i] = [req.body[i]];
      delete req.body[i];
    }
  }
  // Add transfered file entries in req.files (Array).
  req.files = _files;
};

module.exports = function() {
  return new FileParser();
};
