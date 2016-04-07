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


var hop = require('hop');
var path = require('path');
var util = require('util');

var PKG_DIR = ENV.PATHS.PKG_DIR;
var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var svcUtils = require(path.join(INCLUDE_DIR, 'common', 'svc_utils.js'));

var auth = require(path.join(INCLUDE_DIR, 'common', 'auth.js'));

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.text_to_speech;
var rosSrvName = svcParams.ros_srv_name;
var audioOutFormat = svcParams.audio_file_format || "wav";
var audioOutPath = ENV.PATHS.SERVICES_CACHE_DIR;
var basenamePrefix = svcParams.audio_file_basename || "tts_";
/* ----------------------------------------------------------------------- */

// Instantiate interface to rosbridge-websocket-server
var ros = new ROS({hostname: ENV.ROSBRIDGE.HOSTNAME, port: ENV.ROSBRIDGE.PORT,
  reconnect: true, onconnection: function(){
    // .
  }
});

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */



/**
 *  [Text-To-Speech], RAPP Platform Front-End Web Service.
 *  Handles client requests for RAPP Platform Text-To-Speech Services.
 *
 *  @function text_to_speech
 *
 *  @param {Object} args - Service input arguments (literal).
 *  @param {String} args.text - Text to perform TTS on.
 *  @param {String} args.language - Language to be used for TTS translation.
 *
 *  @returns {Object} response - JSON HTTPResponse object.
 *    Asynchronous HTTP Response.
 *  @returns {String} response.payload - Data payload field for the audio/speech
 *    data. Data are character-encoded to base64.
 *  @returns {String} response.basename - An optional basename to be used by the clients
 *  @returns {String} response.encoding - This field declares the character
 *    encoding that was used to encode the audio/speech data of the payload
 *    field. Currently only base64 is supported. This field exists for
 *    future extension purposes.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 *
 */
function svcImpl( kwargs )
{
  var request = this;

  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
      auth.authRequest(request, svcParams.name, authSuccess, authFail);

      function authSuccess(user){
        kwargs = kwargs || {};
        var req = new interfaces.client_req();
        var response = new interfaces.client_res();
        var error = '';

        /* Sniff argument values from request body and create client_req object */
        try{
          svcUtils.parseReq(kwargs, req);
        }
        catch(e){
          error = "Service call arguments error";
          response.error = error;
          sendResponse( hop.HTTPResponseJson(response) );
          return;
        }
        /* -------------------------------------------------------------------- */

        // Assign a unique identification key for this service request.
        var unqCallId = randStrGen.createUnique();

        // Rename file. Add uniqueId value
        var filePath = path.join(audioOutPath,
          basenamePrefix + unqCallId + '.' + audioOutFormat
          );

        var rosSvcReq = new interfaces.ros_req();
        rosSvcReq.audio_output = filePath;
        rosSvcReq.language = req.language;
        rosSvcReq.text = req.text;


        function callback(data){
          // Remove this call id from random string generator cache.
          randStrGen.removeCached( unqCallId );
          // Craft client response using ros service ws response.
          var response = parseRosbridgeMsg( data, filePath );
          // Asynchronous response to client.
          sendResponse( hop.HTTPResponseJson(response) );
          // Remove audio file.
          Fs.rmFile(filePath);
        }

        function onerror(e){
          // Remove this call id from random string generator cache.
          randStrGen.removeCached( unqCallId );
          var response = new interfaces.client_res();
          response.error = svcUtils.ERROR_MSG_DEFAULT;
          // Asynchronous response to client.
          sendResponse( hop.HTTPResponseJson(response) );
        }

        ros.callService(rosSrvName, rosSvcReq,
          {success: callback, fail: onerror});

      }

      function authFail(error){
        var response = auth.responseAuthFailed();
        sendResponse(response);
      }

      /***
       *  Timeout this request. Return to client.
       */
      setTimeout(function(){
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
      }, svcParams.timeout);
      /* ----------------------------------------------- */

    }, this);
}


/***
 *  Craft response object.
 *
 */
function parseRosbridgeMsg(rosbridge_msg, audioFilePath)
{
  var error = rosbridge_msg.error;
  var logMsg = 'Returning to client';

  var response = new interfaces.client_res();

  if ( error )
  {
    logMsg += ' ROS service [' + rosSrvName + '] error' +
      ' ---> ' + error;
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
