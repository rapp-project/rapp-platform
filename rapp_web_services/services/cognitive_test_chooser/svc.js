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
 * [Cognitive-Test-Chooser] RAPP Platform front-end web service.
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

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.cognitive_test_chooser;
var rosSrvName = svcParams.ros_srv_name;
/* ----------------------------------------------------------------------- */

// Initiate communication with rosbridge-websocket-server
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
 *  [Cognitive-Test-Chooser] RAPP Platform front-end web service.
 *  <p>Serves requests for cognitive-exercise selection</p>
 *
 *  @function cognitive_test_chooser
 *
 *  @param {Object} args - Service input arguments (literal).
 *  @param {String} args.user - Registered username.
 *  @param {String} args.test_type - Exercise test-type to request.
 *
 *  Currently available test types are:
 *  <ul>
 *    <li>ArithmericCts</li>
 *    <li>ReasoningCts</li>
 *    <li>AwarenessCts</li>
 *  </ul>
 *    By passing an empty string (''), the RAPP Platform Cognitive Exercises
 *    System is responsible to return a test based on previous user's
 *    performances.
 *
 *
 *  @returns {Object} response - JSON HTTPResponse Object.
 *    Asynchronous HTTP Response.
 *  @returns {String} response.lang - Language.
 *  @returns {Array} response.questions - Vector of questions, for selected
 *    exercise.
 *  @returns {Array} response.possib_ans - Array of possible answers, for
 *    selected exercise.
 *  @returns {Array} response.correct_ans - Vector of correct answers, for
 *    selected exercise.
 *  @returns {String} response.test_instance - Selected Exercise's
 *    test instance name.
 *  @returns {String} response.test_type - Test-type of selected exercise.
 *  @returns {String} response.test_subtype - Test-subtype of selected
 *    exercise.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 *
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

      if( ! req.user ){
        error = 'Empty \"user\" field';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }

      // Assign a unique identification key for this service request.
      var unqCallId = randStrGen.createUnique();

      var rosSvcReq = new interfaces.ros_req();
      rosSvcReq.username = req.user;
      rosSvcReq.testType = req.test_type;


      /***
       * Declare the service response callback here!!
       * This callback function will be passed into the rosbridge service
       * controller and will be called when a response from rosbridge
       * websocket server arrives.
       */
      function callback(data){
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = parseRosbridgeMsg( data );
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
      }

      /***
       * Declare the onerror callback.
       * The onerror callack function will be called by the service
       * controller as soon as an error occures, on service request.
       */
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
 *  @returns {Object} response - Response Object.
 *  @returns {String} response.lang - Language.
 *  @returns {Array} response.questions - Vector of questions, for selected
 *    exercise.
 *  @returns {Array} response.possib_ans - Array of possible answers, for
 *    selected exercise.
 *  @returns {Array} response.correct_ans - Vector of correct answers, for
 *    selected exercise.
 *  @returns {String} response.test_instance - Selected Exercise's
 *    test instance name.
 *  @returns {String} response.test_type - Test-type of selected exercise.
 *  @returns {String} response.test_subtype - Test-subtype of selected
 *    exercise.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var trace = rosbridge_msg.trace;
  var success = rosbridge_msg.success;
  var error = rosbridge_msg.error;
  var questions = rosbridge_msg.questions;
  var answers = rosbridge_msg.answers;
  var correctAnswers = rosbridge_msg.correctAnswers;
  var test = rosbridge_msg.test;
  var testType = rosbridge_msg.testType;
  var testSubType = rosbridge_msg.testSubType;
  var language = rosbridge_msg.language;

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  response.questions = questions;
  response.correct_ans = correctAnswers;
  response.test_instance = test;
  response.test_type = testType;
  response.test_subtype = testSubType;
  response.lang = language;
  response.error = error;

  for (var ii = 0; ii < answers.length; ii++)
  {
    response.possib_ans.push( answers[ii].s );
  }

  return response;
}


module.exports = svcImpl;
