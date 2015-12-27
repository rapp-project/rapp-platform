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
 * [Rapp-Platform-Status] RAPP Platform front-end web service. HTML response.
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

var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );


var testDataPath = path.join(__dirname, '..', '..', 'rapp_testing_tools',
  'scripts', 'default_tests', 'test_data');

var __servicesCacheDir = Fs.resolvePath( ENV.PATHS.SERVICES_CACHE_DIR );
var __serverCacheDir = Fs.resolvePath( ENV.PATHS.SERVER_CACHE_DIR );

var VIEW = require( path.join(__dirname, '..', 'gui', 'src', 'platform_status',
    'view.js') );

var TESTS = require( path.join(__dirname, '..', 'tests', 'tests.js') );

// Initiate communication with rosbridge-websocket-server
var ros = new ROS({hostname: ENV.ROSBRIDGE.HOSTNAME, port: ENV.ROSBRIDGE.PORT,
  reconnect: true, onconnection: function(){
    // .
  }
});


service rapp_platform_status(  )
{
  return VIEW.INDEX();
}


service active_ros_nodes(){
  return hop.HTTPResponseAsync( function( sendResponse ){
    ros.getNodes(
      function(rosNodes){
        sendResponse( hop.HTTPResponseJson({ros_nodes: rosNodes}) );
      },
      {
        fail: function(e){
          console.log(e);
          sendResponse( hop.HTTPResponseJson({ros_nodes: [], error: e}) );
        }
      }
    );
  }, this)
}


service active_ros_topics(){
  return hop.HTTPResponseAsync( function( sendResponse ){
    ros.getTopics(
      function(rosTopics){
        sendResponse( hop.HTTPResponseJson({ros_topics: rosTopics}) );
      },
      {
        fail: function(e){
          console.log(e);
          sendResponse( hop.HTTPResponseJson({ros_topics: [], error: e}) );
        }
      }
    );
  }, this)
}


service active_ros_services(){
  return hop.HTTPResponseAsync( function( sendResponse ){
    ros.getServices(
      function(rosSrvs){
        sendResponse( hop.HTTPResponseJson({ros_srvs: rosSrvs}) );
      },
      {
        fail: function(e){
          console.log(e);
          sendResponse( hop.HTTPResponseJson({ros_srvs: [], error: e}) );
        }
      }
    );
  }, this)
}



service exec_test(srvName)
{
  var response = {success: false, output: undefined, input: undefined};
  console.log('[Exec-Test]: Invoking test for service --> ' +
    srvName);
  if(srvName in srvMap){
    var results = srvMap[srvName]();
    response = {
      success: results.success,
      output: results.output,
      input: results.input
    };
  }
  else{
    response = {
      success: false,
      output: {error: 'Currently Unavailable'},
      input: {error: 'Currently Unavailable'},
    };
  }
  return response;
}


var srvMap = {
  'ontology_subclasses_of': TESTS.ONTOLOGY_SUBCLASSES,
  'ontology_superclasses_of': TESTS.ONTOLOGY_SUPERCLASSES,
  'ontology_is_subsuperclass_of': TESTS.ONTOLOGY_IS_SUBSUPERCLASS,
  'cognitive_test_chooser': TESTS.COGNITIVE_TEST_CHOOSER,
  'record_cognitive_test_performance': TESTS.RECORD_COGNITIVE_TEST_PERFORMANCE,
  'set_noise_profile': TESTS.SET_NOISE_PROFILE,
  'speech_detection_sphinx4': TESTS.SPEECH_DETECTION_SPHINX4,
  'speech_detection_google': TESTS.SPEECH_DETECTION_GOOGLE,
  'face_detection': TESTS.FACE_DETECTION,
  'qr_detection': TESTS.QR_DETECTION,
  'text_to_speech': TESTS.TEXT_TO_SPEECH,
  'available_services': TESTS.AVAILABLE_SERVICES,
  'rapp_user_info': TESTS.RAPP_USER_INFO
};


/****************************************************************************/

function service_url(srvName){
  return 'http://' + hop.hostname + ':' + hop.port + '/hop/' + srvName;
}

