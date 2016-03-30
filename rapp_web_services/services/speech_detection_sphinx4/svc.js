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
 * [Speech-detection-sphinx4] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var hop = require('hop');
var path = require('path');
var util = require('util');


var PKG_DIR = ENV.PATHS.PKG_DIR;
var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var svcUtils = require(path.join(INCLUDE_DIR, 'common',
    'svc_utils.js'));

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.speech_detection_sphinx4;
var SERVICE_NAME = svcParams.name;
var rosSrvName = svcParams.ros_srv_name;

var SERVICES_CACHE_DIR = ENV.PATHS.SERVICES_CACHE_DIR;
var SERVER_CACHE_DIR = ENV.PATHS.SERVER_CACHE_DIR;
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
 *  [Speech-Detection-Sphinx4] RAPP Platform front-end web service.
 *  <p> Serves requests for Speech-Detection using sphinx4 ASR engine. </p>
 *
 *  @function speech_detecion_sphinx4
 *
 *  @param {Object} args - Service input arguments (object literal).
 *  @param {String} args.file_uri - System uri path of transfered (client) file, as
 *    declared in multipart/form-data post field. The file_uri is handled and
 *    forwared to this service, as input argument, by the HOP front-end server.
 *    Clients are responsible to declare this field in the multipart/form-data
 *    post field.
 *  @param {Array} args.words - Words to search for while performing ASR.
 *  @param {Array} args.sentences - Sentences to use as input to sphinx4 ASR.
 *    (For more information study on sphinx4)
 *  @param {Array} args.grammar - Grammar to use as input to sphinx4 ASR.
 *    (For more information, study on sphinx4)
 *  @param {String} args.audio_source - A value that represents information
 *    for the audio source. e.g "nao_wav_1_ch".
 *  @param {String} args.user. Username.
 *  @param {String} args.language. Language to use for ASR.
 *    <ul>
 *      <li> 'el' --> Greek </li>
 *      <li> 'en' --> English </li>
 *    </ul>
 *
 *  @returns {Object} response - JSON HTTPResponse Object.
 *    Asynchronous HTTP Response.
 *  @returns {Array} response.words. An array of the detected-words.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function svcImpl( kwargs )
{
  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
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

      if( ! req.file.length ){
        error = 'No audio file received';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( ! req.audio_source ){
        error = 'Emptry \"audio_source\" argument';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( ! req.user ){
        error = 'Emptry \"user\" argument';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( ! req.language ){
        error = 'Emptry \"language\" argument';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( ! req.words.length ){
        error = 'Emptry \"words\" array argument';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }

      /***
       *  For security reasons, if file_uri is not defined under the
       *  server_cache_dir do not operate. HOP server stores the files under the
       *  __serverCacheDir directory.
       */
      if( req.file[0].indexOf(SERVER_CACHE_DIR) === -1 )
      {
        var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
          " Abortion for security reasons.";
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      /* ----------------------------------------------------------------------- */

      // Assign a unique identification key for this service call.
      var unqCallId = randStrGen.createUnique();

      var cpFilePath = '';

      try{
        cpFilePath = svcUtils.cpInFile(req.file[0], ENV.PATHS.SERVICES_CACHE_DIR,
          unqCallId);
      }
      catch(e){
        console.log(e);
        Fs.rmFile(req.file[0]);
        randStrGen.removeCached(unqCallId);

        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      /*-------------------------------------------------------------------------*/


      var rosSvcReq = new interfaces.ros_req();
      rosSvcReq.path = cpFilePath;
      rosSvcReq.audio_source = req.audio_source;
      rosSvcReq.user = req.user;
      rosSvcReq.language = req.language;
      rosSvcReq.words = req.words;
      rosSvcReq.sentences = req.sentences;
      rosSvcReq.grammar = req.grammar;


      function callback(data){
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = parseRosbridgeMsg( data );
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
      }

      function onerror(e){
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        // craft error response
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
      }

      ros.callService(rosSrvName, rosSvcReq,
        {success: callback, fail: onerror});

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
 * Crafts response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response object.
 *  @returns {Array} response.words. An array of the detected-words.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var words = rosbridge_msg.words;
  var error = rosbridge_msg.error;

  var logMsg = 'Returning to client.';

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  for (var ii = 0; ii < words.length; ii++)
  {
    response.words.push( words[ii] );
  }

  return response;
}


registerSvc(svcImpl, svcParams);
