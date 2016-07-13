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


/***
 * @fileOverview
 *
 * [Text-to-Speech] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');

var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var fs = require('fs');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_text_to_speech_espeak/text_to_speech_topic";
var audioOutFormat = "wav";
var audioOutPath = ENV.PATHS.SERVICES_CACHE_DIR;
var basenamePrefix = "tts_";


/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */



/**
 *  [Text-To-Speech], RAPP Platform Front-End Web Service.
 *  Handles client requests for RAPP Platform Text-To-Speech Services.
 *
 *  Service Implementation
 *
 */
function svcImpl ( req, resp, ros )
{
  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();

  // Rename file. Add uniqueId value
  var filePath = path.join(audioOutPath,
    basenamePrefix + unqCallId + '.' + audioOutFormat);

  var rosMsg = new interfaces.ros_req();
  rosMsg.audio_output = filePath;
  rosMsg.language = req.body.language;
  rosMsg.text = req.body.text;

  // ROS-Service response callback.
  function callback(data){
    // Remove this call id from random string generator cache.
    randStrGen.removeCached( unqCallId );
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data, filePath );
    resp.sendJson(response);
  }

  // ROS-Service onerror callback.
  function onerror(e){
    // Remove local file immediately.
    Fs.rmFile(filePath);
    // Remove this call id from random string generator cache.
    randStrGen.removeCached( unqCallId );
    var response = new interfaces.client_res();
    response.error = e;
    resp.sendJson(response);
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg, callback, onerror);
}


/***
 *  Craft response object.
 *
 */
function parseRosbridgeMsg(rosbridge_msg, audioFilePath)
{
  var error = rosbridge_msg.error;
  var response = new interfaces.client_res();

  if ( error )
  {
    response.error = error;
    return response;
  }

  if( (audioFile = Fs.readFileSync(audioFilePath)) )
  {
    response.payload = audioFile.data.toString('base64');
    response.basename = audioFile.basename;
    response.encoding = 'base64';
    // Remove local file immediately.
    Fs.rmFile(audioFilePath);
  }

  return response;
}


module.exports = svcImpl;
