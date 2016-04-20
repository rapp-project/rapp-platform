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


function FileParser(fields) {
  this.fields = fields || ['file'];
}


FileParser.prototype.parse = function(req) {
  var files = [];

  for (var i in this.fields){
    if(! req.body[this.fields[i]]){
      continue;
    }
    else if (req.body[this.fields[i]] instanceof Array) {
      //An Array of files
      files.concat(req.body[thisfields[i]]);
      // Remove from req.body
      delete req.body[this.fields[i]];
    }
    else{
      // Meaning single file.
      files.push(req.body[this.fields[i]]);
      // Remove from req.body
      delete req.body[this.fields[i]];
    }
  }
  // Add transfered file entries in req.files (Array).
  req.files = files;
};

module.exports = function(fields) {
  return new FileParser(fields);
};
