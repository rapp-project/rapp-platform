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

var svcUtils = require(path.join(INCLUDE_DIR, 'common',
    'svc_utils.js'));

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.text_to_speech;
var SERVICE_NAME = svcParams.name;
var rosSrvName = svcParams.ros_srv_name;
var audioOutFormat = svcParams.audio_file_format;
var audioOutPath = ENV.PATHS.SERVICES_CACHE_DIR;
var basenamePrefix = svcParams.basename;
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

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = svcParams.timeout; // ms
var maxTries = svcParams.retries;
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
  var req = new interfaces.client_req();
  var response = new interfaces.client_res();
  var error = '';

  kwargs = kwargs || {};
  for( var i in req ){
    req[i] = (kwargs[i] !== undefined) ? kwargs[i] : req[i];
  }
  if ( ! req.text ){
    error = 'Empty \"text\" field';
    return hop.HTTPResponseJson(response);
  }
  if ( ! req.language ){
    error = 'Empty \"language\" field';
    response.error = error;
    return hop.HTTPResponseJson(response);
  }

  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();

  var startT = new Date().getTime();
  var execTime = 0;

  // Rename file. Add uniqueId value
  var filePath = path.join(audioOutPath,
    basenamePrefix + unqCallId + '.' + audioOutFormat
    );

  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /***
       *  Status flags.
       */
      var respFlag = false;
      var retClientFlag = false;
      var wsError = false;
      var retries = 0;
      /* --------------------------------------------------- */

      var rosSvcReq = new interfaces.ros_req();
      rosSvcReq.audio_output = filePath;
      rosSvcReq.language = req.language;
      rosSvcReq.text = req.text;

      /***
       * Declare the service response callback here!!
       * This callback function will be passed into the rosbridge service
       * controller and will be called when a response from rosbridge
       * websocket server arrives.
       */
      function callback(data){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        //console.log(data);

        // Craft client response using ros service ws response.
        var response = parseRosbridgeMsg( data, audioOutPath );
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }

      /***
       * Declare the onerror callback.
       * The onerror callack function will be called by the service
       * controller as soon as an error occures, on service request.
       */
      function onerror(e){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }

      /* -------------------------------------------------------- */

      // Invoke ROS-Service request.
      ros.callService(rosSrvName, rosSvcReq,
        {success: callback, fail: onerror});

      /***
       *  Set Timeout wrapping function.
       *  Polling in defined time-cycle. Catch timeout connections etc...
       */
      function asyncWrap(){
        setTimeout( function(){

         /***
          *  If received message from rosbridge websocket server or an error
          *  on websocket connection, stop timeout events.
          */
          if ( respFlag || wsError || retClientFlag ) { return; }

          retries += 1;

          var logMsg = 'Reached rosbridge response timeout' + '---> [' +
            timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
            'Retry-' + retries;

          /***
           * Fail. Did not receive message from rosbridge.
           * Return to client.
           */
          if ( retries >= maxTries )
          {
            logMsg = 'Reached max_retries [' + maxTries + ']' +
              ' Could not receive response from rosbridge...';

            execTime = new Date().getTime() - startT;

            var response = new interfaces.client_res();
            response.error = svcUtils.ERROR_MSG_DEFAULT;
            sendResponse( hop.HTTPResponseJson(response));
            retClientFlag = true;
            return;
          }
          /*--------------------------------------------------------*/
          asyncWrap();

        }, timeout);
      }
      asyncWrap();
      /*=================================================================*/
    }, this );
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
    response.error = svcUtils.ERROR_MSG_DEFAULT;
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
  //console.log(crafted_msg)

  return response;
}


