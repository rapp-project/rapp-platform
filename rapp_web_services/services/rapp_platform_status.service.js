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

var __includeDir = path.join(__dirname, '..', 'modules');
var __configDir = path.join(__dirname, '..', 'config');

var Fs = require( path.join(__includeDir, 'fileUtils.js') );

var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );

var pathsEnv = require( path.join(__configDir, 'env', 'paths.json') );
var rosbridgeEnv = require( path.join(__configDir, 'env', 'rosbridge.json' ) );

var testDataPath = path.join(__dirname, '..', '..', 'rapp_testing_tools',
  'scripts', 'default_tests', 'test_data');

var __serverCacheDir = Fs.resolve_path( pathsEnv.cache_dir_server );

import service available_services();

// Initiate connection to rosbridge_websocket_server
var ros = new ROS({hostname: '', port: '', reconnect: true, onconnection:
  function(){
    // .
  }
});

var platformActiveServices = [];
var platformActiveNodes = [];
var platformActiveTopics = [];
var platformHopServices = [];
var rappRosServices  = [];
var pollCycle = 1000; // 10 sec


loopInf();


function navbar(){
  return <div class="navbar navbar-inverse navbar-fixed-top">
    <div class="navbar-inner">
      <div class="container">
        <button type="button" class="btn btn-navbar" data-toggle="collapse" data-target=".nav-collapse">
          <a class="btn btn-success" href="#">RAPP Platform Status</a>
        </button>
      </div>
    </div>
  </div>;
}


function fillLabel(labelName){
  var lblText = '';
  var contClass = '';
  switch(labelName){
    case "ros-nodes": 
      lblText = "RAPP Platform ROS-Nodes";
      contClass =
        (platformActiveNodes.length > 1) ? "bg-success" : "bg-danger";
      break;
    case "ros-services":
      lblText = "RAPP Platform ROS-Services";
      contClass =
        (platformActiveServices.length > 1) ? "bg-success" : "bg-danger";
      break;
    case "ros-topics":
      lblText = "RAPP Platform ROS-Topics";
      contClass =
        (platformActiveTopics.length > 1) ? "bg-success" : "bg-danger";
      break;
    case "hop-services":
      lblText = "RAPP Platform HOP web services";
      contClass= "bg-info";
      break;
    default:
      break;
  }

  return <h2 align="center">
    <p class=${contClass} align="center">${lblText}</p>
    </h2>
  ;
}


function headerHtml(){
  return  <head>
    ~{ var selectedHopSrv = ''; }
    <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.4/css/bootstrap.min.css">
    <meta charset="utf-8"/>
    <meta name="viewpoint" content="width=device-width, initial-scale=1">
    <style>
      body {
        padding-top: 60px; /* 60px to make the container go all the way to the bottom of the topbar */
      }
    </style>
  </head>
  ; 
}


service rapp_platform_status(  )
{
  var rapp_srv_literal = '';
  var services_literal = '';
  var nodes_literal = '';
  var topics_literal = '';
  var hop_srv_literal = '';

  for(var index = 0 ; index < rappRosServices.length ; index++)
  {
    rapp_srv_literal += "<option>" +   rappRosServices[index] + '</option>';
  }

  for(var index = 0 ; index < platformActiveNodes.length ; index++)
  {
    nodes_literal += "<option>" + platformActiveNodes[index] + '</option>';
  }

  for(var index = 0 ; index < platformActiveNodes.length ; index++)
  {
    topics_literal += "<option>" + platformActiveTopics[index] + '</option>';
  }

  for(var index = 0 ; index < platformHopServices.length ; index++)
  {
    hop_srv_literal += "<option>" + platformHopServices[index] + '</option>';
  }


  return <html lang="en">
    ${headerHtml()}
    <body>
      ${navbar()}
      <div class='jumbotron' align='center'>
        <h1>RAPP Platform Status</h1>
      </div>
      <div class='col-md-4' style='word-wrap:break-word;'>
        <div class='row-fluid'>
          ${fillLabel("ros-services")}
          <select id='SrvBox' class="form-control" onchange=~{
            var selected = this.options[this.selectedIndex].value;
            }>
            ${rapp_srv_literal}
          </select>
        </div>
        <div class='row-fluid'>
          ${fillLabel("ros-nodes")}
          <select id='NodesBox' class="form-control" onchange=~{
            var selected = this.options[this.selectedIndex].value
            }>
            ${nodes_literal}
          </select>
        </div>
        <div class='row-fluid'>
          ${fillLabel("ros-topics")}
          <select id='TopicsBox' class="form-control" onchange=~{
            var selected = this.options[this.selectedIndex].value
            }>
            ${topics_literal}
          </select>
        </div>
      </div>
      <div class='container-fluid'>
      <div class='col-md-4 text-centered' style='word-wrap:break-word;'>
        <div class='row-fluid'>
          ${fillLabel("hop-services")}
          <select id='HopSrvBox' class="form-control" onchange=~{
            selectedHopSrv = this.options[this.selectedIndex].value
            }>
            ${hop_srv_literal}
          </select>
        </div>
        <div class='row-fluid'>
          <button type='button' class='btn btn-primary btn-block btn-lg' onclick=~{
            var callSrv = selectedHopSrv;
            var retObj = ${testService}(callSrv).postSync();
            //console.log(retObj)
            var success = retObj.success;
            var srvResponse = retObj.output;
            var inputParams = retObj.input;
            var alertClass = '"alert alert-info"';
            var alertMessage = '';
            if(success){
              alertClass = '"alert alert-success"';
              alertMessage = 'Successful Service Call!';
            }
            else{
              alertClass = '"alert alert-danger"';
              alertMessage = 'Failure on Service Call!';
            }

            document.getElementById('hopResults').innerHTML =
              '<div class="alert alert-info"><strong>Testing service ' + selectedHopSrv + '</strong><br>'
            document.getElementById('hopSuccess').innerHTML =
              '<div class=' + alertClass + '>' + alertMessage + '</div>';

            var res_literal = '<div class="alert alert-info"><strong>Testing service ' +
              selectedHopSrv + '<br><br></strong><strong>Input</strong>' +
              '<li>' + JSON.stringify(inputParams) + '</li>';
            res_literal += '<br><strong>Output</strong>';
            res_literal += '<li>' + JSON.stringify(srvResponse) + '</li>'
            res_literal += '</div>';
            document.getElementById('hopResults').innerHTML = res_literal;

            }>
            Call Hop Service
          </button>
        </div>
      </div>
      <div class='col-md-4 text-centered' style='word-wrap:break-word;'>
        <div class='row-fluid' id='hopResults'></div>
        <div class='row-fluid' id='empty'></div>
        <div class='row-fluid' id='hopSuccess'></div>
      </div>
      </div>
    </body>
  </html>;
}


service testService(srvName)
{
  var response = {success: false, output: undefined, input: undefined};
  console.log('[Rapp-Platform-Status]: Invoking test for service --> ' +
    srvName)
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
      output: 'Service call error! Service does not exist',
      input: 'Service call error! Service does not exist',
    };
  }
  return response;
}


var srvMap = {
  'ontology_subclasses_of': test_ontology_subclassesOf,
  'ontology_superclasses_of': test_ontology_superclassesOf,
  'ontology_is_subsuperclass_of': test_ontology_is_subsuperclassOf,
  'set_noise_profile': test_denoise_profile,
  'speech_detection_sphinx4': test_sphinx4,
  'speech_detection_google': test_speech_detection_google,
  'face_detection': test_face_detection,
  'qr_detection': test_qr_detection,
  'text_to_speech': test_tts,
  'record_cognitive_test_performance': test_record_cognitive_performance,
  'cognitive_test_chooser': test_cognitive_test_chooser
}



/*****************************************************************************
 *                          Per service Tests.
 *****************************************************************************
 */


function test_ontology_subclassesOf(){
  import service ontology_subclasses_of();
  var args = {
    query: 'Oven'
  };
  var success = true;
  var response = ontology_subclasses_of(args).postSync();
  var valid_results = [
    'http://knowrob.org/kb/knowrob.owl#MicrowaveOven',
    'http://knowrob.org/kb/knowrob.owl#RegularOven',
    'http://knowrob.org/kb/knowrob.owl#ToasterOven'
  ]

  for(i in response.results){
    if( valid_results.indexOf(response.results[i]) > -1 ) {
      continue;
    }
    else{
      success = false;
      break;
    }
  }
  return {success: success, output: response.results, input: args};
}


function test_ontology_superclassesOf(){
  import service ontology_superclasses_of();
  var args = {
    query: 'Oven'
  }
  var response = ontology_superclasses_of(args).postSync();
  var success = true;
  var valid_results = [
    'http://knowrob.org/kb/knowrob.owl#Box-Container',
    'http://knowrob.org/kb/knowrob.owl#FurniturePiece',
    'http://knowrob.org/kb/knowrob.owl#HeatingDevice',
    'http://knowrob.org/kb/knowrob.owl#HouseholdAppliance'
  ]

  for(i in response.results){
    if( valid_results.indexOf(response.results[i]) > -1 ) {
      continue;
    }
    else{
      success = false;
      break;
    }
  }
  return {success: success, output: response.results, input: args};
}


function test_ontology_is_subsuperclassOf(){
  var success = true;
  var results = undefined;
  var s = 'ontology_is_subsuperclass_of';
  var args = {
    'parent_class': 'SpatialThing',
    'child_class': 'Oven',
    'recursive': true
  }
  var validResult = true;
  var response = undefined;
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
  var testFileSrc = testDataPath + 'Lenna.png';
  var testFileDest = __serverCacheDir + 'status_test_lenna.png';
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

function loopInf(){
  setTimeout(function(){
    ros.getServices(function(data){
      platformActiveServices = data;
      //console.log(data)
      rappRosServices = [];
      for(i in platformActiveServices){
        if(platformActiveServices[i].indexOf("rapp") > -1){
          rappRosServices.push(platformActiveServices[i])
        }
      }
    })
    ros.getNodes(function(data){
      platformActiveNodes = data;
    })
    ros.getTopics(function(data){
      platformActiveTopics = data;
    })
    available_services().post( function( response ){
      platformHopServices = response.services;
      //console.log(response.services)
    })

    loopInf();
  }, pollCycle)
}


