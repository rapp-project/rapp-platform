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
  return VIEW.INDEX;
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
  'ontology_is_subsuperclass_of': TESTS.ONTOLOGY_IS_SUBSUPERCLASS
  //'set_noise_profile': test_denoise_profile,
  //'speech_detection_sphinx4': test_sphinx4,
  //'speech_detection_google': test_speech_detection_google,
  //'face_detection': test_face_detection,
  //'qr_detection': test_qr_detection,
  //'text_to_speech': test_tts,
  //'record_cognitive_test_performance': test_record_cognitive_performance,
  //'cognitive_test_chooser': test_cognitive_test_chooser
};



/*****************************************************************************
 *                          Per service Tests.
 *****************************************************************************
 */


function test_denoise_profile(){
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'set_noise_profile';
  var testFileSrc = testDataPath + 'denoise_source.wav';
  var testFileDest = __serverCacheDir + 'status_test_denoise.wav';
  Fs.copyFile(testFileSrc, testFileDest);
  var args = {
    'file_uri': testFileDest,
    'audio_source': 'nao_wav_1_ch',
    'user': 'rapp'
  }
  var webService = hop.webService(service_url(s));
  var srv = webService(args);
  try{
    response = srv.postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {results = response}
  return {success: success, output: results, input: args};
}


function test_sphinx4(){
  import service speech_detection_sphinx4();
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'speech_detection_sphinx4';
  var testFileSrc = testDataPath + 'yes-no.wav';
  var testFileDest = __serverCacheDir + 'status_test_sphinx.wav';
  Fs.copyFile(testFileSrc, testFileDest);
  var args = {
    language: 'en',
    audio_source: 'nao_wav_1_ch',
    words: ['yes', 'no'],
    sentences: ['yes', 'no'],
    grammar: [],
    user: 'rapp',
    file_uri: testFileDest
  }
  try{
    response = speech_detection_sphinx4(args).postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {results = response}
  return {success: success, output: results, input: args};
}


function test_speech_detection_google(){
  import service speech_detection_google();
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'speech_detection_google';
  var testFileSrc = testDataPath + 'speech_detection_samples/' +
    'recording_sentence1.ogg';
  var testFileDest = __serverCacheDir + 'status_test_google.ogg';
  Fs.copyFile(testFileSrc, testFileDest);
  var args = {
    language: 'en',
    audio_source: 'nao_ogg',
    user: 'rapp',
    file_uri: testFileDest
  }
  try{
    response = speech_detection_google(args).postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {results = response}
  return {success: success, output: results, input: args};

}


function test_face_detection(){
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'face_detection';
  var testFileSrc = path.join(testDataPath, 'Lenna.png');
  var testFileDest = path.join(__serverCacheDir, 'status_test_lenna.png');
  Fs.copyFile(testFileSrc, testFileDest);
  var args = {
    'file_uri': testFileDest,
  }
  var webService = hop.webService(service_url(s));
  var srv = webService(args);
  try{
    response = srv.postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {results = response}
  return {success: success, output: results, input: args};
}


function test_qr_detection(){
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'qr_detection';
  var testFileSrc = testDataPath + 'qr_code_rapp.jpg';
  var testFileDest = __serverCacheDir + 'status_test_qr.jpg';
  Fs.copyFile(testFileSrc, testFileDest);
  var args = {
    'file_uri': testFileDest,
  }
  var webService = hop.webService(service_url(s));
  var srv = webService(args);
  try{
    response = srv.postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {results = response}
  return {success: success, output: results, input: args};
}

function test_tts(){
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'text_to_speech';
  var args = {
    'language': 'el',
    'text': 'Καλησπέρα'
  }
  var webService = hop.webService(service_url(s));
  var srv = webService(args);
  try{
    response = srv.postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {
    results = response;
    results['payload'] = 'This is the hidden payload field!!';
  }
  return {success: success, output: results, input: args};
}


function test_record_cognitive_performance(){
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'record_cognitive_test_performance';
  var args = {
    'user': 'rapp',
    'test': 'ArithmeticCts_obzxzwaP',
    'score': 50
  }
  var webService = hop.webService(service_url(s));
  var srv = webService(args);
  try{
    response = srv.postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {results = response;}
  return {success: success, output: results, input: args};
}


function test_cognitive_test_chooser(){
  var success = true;
  var results = undefined;
  var response = undefined;
  var s = 'cognitive_test_chooser';
  var args = {
    'user': 'rapp',
    'testType': 'ArithmeticCts'
  }
  var webService = hop.webService(service_url(s));
  var srv = webService(args);
  try{
    response = srv.postSync();
  }
  catch(e){
    console.log(e);
    results = e;
    success = false;
  }
  if(success) {results = response;}
  return {success: success, output: results, input: args};
}

/****************************************************************************/

function service_url(srvName){
  return 'http://' + hop.hostname + ':' + hop.port + '/hop/' + srvName;
}

