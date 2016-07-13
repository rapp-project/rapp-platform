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
 * [Cognitive-Test-Selector] RAPP Platform web service implementation.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 */


var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_cognitive_exercise/cognitive_exercise_chooser";


/**
 *  [Cognitive-Test-Selector]
 *  Handles requests to cognitive_test_chooser RAPP Platform Service
 *
 *  Service Implementation.
 *
 *
 */
function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();
  rosMsg.username = req.username;
  rosMsg.testType = req.body.test_type;
  rosMsg.testSubType = req.body.test_subtype;
  rosMsg.testDifficulty = req.body.test_diff;
  rosMsg.testIndex = req.body.test_index;

  // ROS-Service response callback.
  function callback(data){
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  // ROS-Service onerror callback.
  function onerror(e){
    var response = new interfaces.client_res();
    response.error = e;
    resp.sendJson(response);
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg, callback, onerror);
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
