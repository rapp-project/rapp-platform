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

var path = require('path');
var fs = require('fs');

var pkgPath = path.join(__dirname, '../..');
var Fs = require( path.join(pkgPath, 'modules/common', 'fileUtils.js') );
var ENV = require( path.join(pkgPath, 'env.js') );
var testParams = require( path.join(__dirname, 'params.json') );


exports.TEST = function (){
  import service speech_detection_sphinx4();

  var serverCacheDir = ENV.PATHS.SERVER_CACHE_DIR;
  var success = false;
  var response = {};
  var validResponse = testParams.response;
  var args = {};

  for( var arg in testParams.request.args ){
    args[arg] = testParams.request.args[arg];
  }

  args.file_uri = Fs.resolvePath( args.file_uri );

  var fileBasename = path.basename( args.file_uri );
  var destPath = path.join( Fs.resolvePath( serverCacheDir ), fileBasename );


  if( Fs.copyFile(args.file_uri, destPath) ){
    args.file_uri = destPath;
    response = speech_detection_sphinx4(args).postSync();

    if( validResponse.error === response.error ){
      success = true;
      for( var i in validResponse.words ){
        if( response.words[i] !== validResponse.words[i]){
          success = false;
          break;
        }
      }
    }
  }

  return {success: success, output: response, input: args};
};

