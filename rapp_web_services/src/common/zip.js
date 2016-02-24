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
 * @description Zip/Unzip wrapper methods
 *
 * @author Konstantinos Panayiotou
 * @copyright Rapp Project EU 2016
 *
 */

var AdmZip;
var RAND_STR_LENGTH = 5;

try{
  AdmZip = require('adm-zip');
}
catch( e ){
  console.error("Could not load node-unzip: " + e);
}

var fs = require('fs');
var path = require('path');

var RandStrGen = require(path.join(__dirname, 'randStringGen.js'));
var strGen = new RandStrGen( RAND_STR_LENGTH );

var Fs = require(path.join(__dirname, 'fileUtils.js'));


function _unzip( zipFilepath, outPath ){
  if(! outPath){
    var ext = strGen.createUnique();
    outPath = path.join(path.dirname(zipFilepath), ext);
  }
  //var readStream = fs.createReadStream(zipFilepath);
  //var writeStream = fs.createWriteStream(outPath);
  try{
    var zip = new AdmZip(zipFilepath);
    zip.extractAllTo(outPath);
  }
  catch(e){
    console.error(e);
    return false;
  }
  var files = Fs.lsSync(outPath);
  var filepaths = [];
  for( var i in files ){
    filepaths.push(path.join(outPath, files[i]));
  }
  var stat = {
    files: files,
    outpath: outPath,
    filepaths: filepaths
  };
  return stat;
}


function isZipFile( zipFilepath ){
  try{
    var zip = new AdmZip(zipFilepath);
  }
  catch(e){
    return false;
  }
  return true;
}

module.exports = {
  unzip: _unzip,
  isZipFile: isZipFile
};
