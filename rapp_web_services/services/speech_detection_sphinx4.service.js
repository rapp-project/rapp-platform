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


var __DEBUG__ = false;

var hop = require('hop');
var path = require('path');

var ENV = require( path.join(__dirname, '..', 'env.js') );

var __includeDir = path.join(__dirname, '..', 'modules');
var __configDir = path.join(__dirname, '..', 'config');

var Fs = require( path.join(__includeDir, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(__includeDir, 'common',
    'randStringGen.js') );

var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );


/* ------------< Load and set global configuration parameters >-------------*/
var __hopServiceName = 'speech_detection_sphinx4';
var __hopServiceId = null;
var __servicesCacheDir = Fs.resolvePath( ENV.PATHS.SERVICES_CACHE_DIR );
var __serverCacheDir = Fs.resolvePath( ENV.PATHS.SERVER_CACHE_DIR );
/* ----------------------------------------------------------------------- */

var rosSrvName = ENV.SERVICES[__hopServiceName].ros_srv_name;

// Initiate communication with rosbridge-websocket-server
var ros = new ROS({hostname: ENV.ROSBRIDGE.HOSTNAME, port: ENV.ROSBRIDGE.PORT,
  reconnect: true, onconnection: function(){
    // .
  }
});

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = ENV.SERVICES[__hopServiceName].timeout; // ms
var maxTries = ENV.SERVICES[__hopServiceName].retries;
/* ----------------------------------------------------------------------- */

var colors = {
  error:    String.fromCharCode(0x1B) + '[1;31m',
  success:  String.fromCharCode(0x1B) + '[1;32m',
  ok:       String.fromCharCode(0x1B) + '[34m',
  yellow:   String.fromCharCode(0x1B) + '[33m',
  clear:    String.fromCharCode(0x1B) + '[0m'
};


// Register communication interface with the master-process
register_master_interface();


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
service speech_detection_sphinx4(
  {file_uri: '', language: '', audio_source: '',
    words: [], sentences: [], grammar: [], user: ''
  })
{
  /***
   *  For security reasons, if file_uri is not defined under the
   *  server_cache_dir do not operate. HOP server stores the files under the
   *  __serverCacheDir directory.
   */
  if( file_uri.indexOf(__serverCacheDir) === -1 )
  {
    var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
        " Abortion for security reasons.";
    postMessage( craft_slaveMaster_msg('log', errorMsg) );
    console.log(colors.error + '[Speech-Detection-Sphinx4]: ' + errorMsg +
      colors.clear);

    var response = {
      words: [],
      error: errorMsg
    };

    return hop.HTTPResponseJson(response);
  }
  /* ----------------------------------------------------------------------- */

  // Assign a unique identification key for this service call.
  var unqCallId = randStrGen.createUnique();

  var startT = new Date().getTime();
  var execTime = 0;

  postMessage( craft_slaveMaster_msg('log', 'client-request {' + rosSrvName +
    '}') );
  var logMsg = 'Audio data stored at [' + file_uri + ']';
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var fileUrl = file_uri.split('/');
  var fileName = fileUrl[fileUrl.length -1];

  var cpFilePath = __servicesCacheDir + fileName.split('.')[0] + '-'  +
    unqCallId + '.' + fileName.split('.')[1];
  cpFilePath = Fs.resolvePath(cpFilePath);
  /* ---------------------------------------------------------------- */


  /* --------------------- Handle transferred file ------------------------- */
  if (Fs.renameFile(file_uri, cpFilePath) === false)
  {
    //could not rename file. Probably cannot access the file. Return to client!
    var logMsg = 'Failed to rename file: [' + file_uri + '] --> [' +
      cpFilePath + ']';

    postMessage( craft_slaveMaster_msg('log', logMsg) );
    Fs.rmFile(file_uri);
    randStrGen.removeCached(unqCallId); // Dismiss the unique identity key
    var response = craft_error_response();
    return hop.HTTPResponseJson(response);
  }
  logMsg = 'Created copy of file ' + file_uri + ' at ' + cpFilePath;
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  /*-------------------------------------------------------------------------*/

  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /***
       *  Status flags.
       */
      var respFlag = false;
      var wsError = false;
      var retClientFlag = false;
      var retries = 0;
      /* --------------------------------------------------- */

      // Fill Ros Service request msg parameters here.
      var args = {
        path: cpFilePath,
        audio_source: audio_source,
        words: isJson(words) ? JSON.parse(words) : words,
        sentences: isJson(sentences) ? JSON.parse(sentences) : sentences,
        grammar: isJson(grammar) ? JSON.parse(grammar) : grammar,
        language: language,
        user: user
      };


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
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = craft_response( data );
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
        // Remove cached file. Release resources.
        Fs.rmFile(cpFilePath);
        // craft error response
        var response = craft_error_response();
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }


      // Invoke ROS-Service request.
      ros.callService(rosSrvName, args,
        {success: callback, fail: onerror});

      /***
       * Set Timeout wrapping function.
       * Polling in defined time-cycle. Catch timeout connections etc...
       */
      function asyncWrap(){
        setTimeout( function(){

         /***
          * If received message from rosbridge websocket server or an error
          * on websocket connection, stop timeout events.
          */
          if ( respFlag || wsError || retClientFlag ) { return; }

          retries += 1;

          var logMsg = 'Reached rosbridge response timeout' + '---> [' +
            timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
            'Retry-' + retries;
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          /***
           * Fail. Did not receive message from rosbridge.
           * Return to client.
           */
          if ( retries >= maxTries )
          {
            logMsg = 'Reached max_retries [' + maxTries + ']' +
              ' Could not receive response from rosbridge...';
            postMessage( craft_slaveMaster_msg('log', logMsg) );

            // Remove cached file. Release resources.
            Fs.rmFile(cpFilePath);

            execTime = new Date().getTime() - startT;
            postMessage( craft_slaveMaster_msg('execTime', execTime) );

            var response = craft_error_response();
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
 * Crafts response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response object.
 *  @returns {Array} response.words. An array of the detected-words.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function craft_response(rosbridge_msg)
{
  var words = rosbridge_msg.words;
  var error = rosbridge_msg.error;

  var logMsg = 'Returning to client.';

  var response = {
    words: [],
    error: ''
  };

  for (var ii = 0; ii < words.length; ii++)
  {
    response.words.push( words[ii] );
  }
  response.error = error;

  if (error !== '')
  {
    logMsg += ' ROS service [' + rosSrvName + '] error' +
      ' ---> ' + error;
  }
  else
  {
    logMsg += ' ROS service [' + rosSrvName + '] returned with success';
  }
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return response;
}



/***
 *  Craft service error response object. Used to return to client when an
 *  error has been occured, while processing client request.
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure';

  var response = {
    words: [],
    error: errorMsg
  };

  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return response;
}


/***
 *  Register interface with the main hopjs process. After registration
 *  this worker service can communicate with the main hopjs process through
 *  websockets.
 *
 *  The global scoped postMessage is used in order to send messages to the main
 *  process.
 *  Furthermore, the global scoped onmessage callback function declares the
 *  handler for incoming messages from the hopjs main process.
 *
 *  Currently log messages are handled by the main process.
 */
function register_master_interface()
{
  // Register onexit callback function
  onexit = function(e){
    console.log("Service [%s] exiting...", __hopServiceName);
    var logMsg = "Received termination command. Exiting.";
    postMessage( craft_slaveMaster_msg('log', logMsg) );
  };

  // Register onmessage callback function
  onmessage = function(msg){
    if (__DEBUG__)
    {
      console.log("Service [%s] received message from master process",
        __hopServiceName);
      console.log("Msg -->", msg.data);
    }

    var logMsg = 'Received message from master process --> [' +
      msg.data + ']';
    postMessage( craft_slaveMaster_msg('log', logMsg) );

    var cmd = msg.data.cmdId;
    var data = msg.data.data;
    switch (cmd)
    {
      case 2055:  // Set worker ID
        __hopServiceId = data;
        break;
      default:
        break;
    }
  };

  // On initialization inform master and append to log file
  var logMsg = "Initiated worker";
  postMessage( craft_slaveMaster_msg('log', logMsg) );
}


/***
 *  Returns master-process comm msg literal.
 */
function craft_slaveMaster_msg(msgId, msg)
{
  var _msg = {
    name: __hopServiceName,
    id:   __hopServiceId,
    msgId: msgId,
    data: msg
  };
  return _msg;
}

/***
 *  Check if input value is in json string representation.
 *
 *  @returns True if isJson.
 */
function isJson(str){
  try{
    JSON.parse(str);
  }
  catch(e){
    return false;
  }
  return true;
}
