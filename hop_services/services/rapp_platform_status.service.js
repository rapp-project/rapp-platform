/*!
 * @file rapp_platform_status.service.js
 * @brief Returns available hop services located on RAPP Platform
 */


/**
 *  MIT License (MIT)
 *
 *  Copyright (c) <2014> <Rapp Project EU>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  Authors: Konstantinos Panayiotou
 *  Contact: klpanagi@gmail.com
 *
 */


/*--------------Load required modules-----------*/
var hop = require('hop');
var module_path = '../modules/';
var config_path = '../config/';
var ROS = require( module_path + 'RosBridgeJS/src/Rosbridge.js' );
var rosbridgeEnv = require( config_path + 'env/rosbridge.json' );
var testDataPath = __dirname +
  '../../rapp_testing_tools/testing_tools/test_data';
/*----------------------------------------------*/

import service available_services();
var ros = new ROS(rosbridgeEnv.ip_addr, rosbridgeEnv.port);

var platformActiveServices = [];
var platformActiveNodes = [];
var platformActiveTopics = [];
var platformHopServices = [];
var rappRosServices  = [];
var pollCycle = 1000; // 10 sec
var refreshRate__ = '30'; // seconds


loopInf();

var navbar =
    '<div class="navbar navbar-inverse navbar-fixed-top">' +
      '<div class="navbar-inner">' +
        '<div class="container">' +
          '<button type="button" class="btn btn-navbar" data-toggle="collapse" data-target=".nav-collapse">' +
            '<a class="btn btn-success" href="#">RAPP Platform Status</a>' +
          '</button>' +
          //'<a class="brand" href="#">Project name</a>' +
        '</div>' +
      '</div>' +
    '</div>';




service rapp_platform_status(  )
{
  var srv_class = (platformActiveServices.length > 1) ? "bg-success" : "bg-danger";
  var nodes_class = (platformActiveNodes.length > 1) ? "bg-success" : "bg-danger";
  var topics_class = (platformActiveTopics.length > 1) ? "bg-success" : "bg-danger";
  var hop_class = "bg-info";

  var rapp_srv_literal = '';
  var services_literal = '';
  var nodes_literal = '';
  var topics_literal = '';
  var hop_srv_literal = '';

  var nodes_label =
    '<h2 align="center"><p class=' + nodes_class +
    ' align="center">ROS Nodes</p></h2>';
  var rapp_services_label =
    '<h2 align="center"><p class=' + srv_class +
    ' align="center">ROS Services</p></h2>';
  var topics_label =
    '<h2 align="center"><p class=' + topics_class +
    ' align="center">ROS Topics</p></h2>';
  var hop_srv_label =
    '<h2 align="center"><p class=' + hop_class +
    ' align="center">HOP Web Services</p></h2>';


  for(index = 0 ; index < rappRosServices.length ; index++)
  {
    rapp_srv_literal += "<option>" +   rappRosServices[index] + '</option>';
  }

  for(index = 0 ; index < platformActiveNodes.length ; index++)
  {
    nodes_literal += "<option>" + platformActiveNodes[index] + '</option>';
  }

  for(index = 0 ; index < platformActiveNodes.length ; index++)
  {
    topics_literal += "<option>" + platformActiveTopics[index] + '</option>';
  }

  for(index = 0 ; index < platformHopServices.length ; index++)
  {
    hop_srv_literal += "<option>" + platformHopServices[index] + '</option>';
  }


  return <html lang="en">
    <head>
      ~{ var selectedHopSrv = ''; }
      <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.4/css/bootstrap.min.css">
      <meta charset="utf-8"/>
      <meta name="viewpoint" content="width=device-width, initial-scale=1">
      <meta http-equiv="refresh" content=${refreshRate__} >
      <style>
        body {
          padding-top: 60px; /* 60px to make the container go all the way to the bottom of the topbar */
        }
      </style>
    </head>
    <body>
      ${navbar}
      <div class='jumbotron' align='center'>
        <h1>RAPP Platform information</h1>
        <div>
          To work, roscore and rosbridge_websocket must be executed
        </div>
      </div>
      <div class='col-md-4' style='word-wrap:break-word;'>
        <div class='row-fluid'>
          ${rapp_services_label}
          <select id='SrvBox' class="form-control" onchange=~{
            var selected = this.options[this.selectedIndex].value
              alert(selected)
            }>
            ${rapp_srv_literal}
          </select>
        </div>
        <div class='row-fluid'>
          ${nodes_label}
          <select id='NodesBox' class="form-control" onchange=~{
            var selected = this.options[this.selectedIndex].value
              //alert(selected)
            }>
            ${nodes_literal}
          </select>
        </div>
        <div class='row-fluid'>
          ${topics_label}
          <select id='TopicsBox' class="form-control" onchange=~{
            var selected = this.options[this.selectedIndex].value
              //alert(selected)
            }>
            ${topics_literal}
          </select>
        </div>
      </div>
      <div class='container-fluid'>
      <div class='col-md-4 text-centered' style='word-wrap:break-word;'>
        <div class='row-fluid'>
          ${hop_srv_label}
          <select id='HopSrvBox' class="form-control" onchange=~{
            selectedHopSrv = this.options[this.selectedIndex].value
              //alert(selected)
            }>
            ${hop_srv_literal}
          </select>
        </div>
        <div class='row-fluid'>
          <button type='button' class='btn btn-primary btn-block btn-lg' onclick=~{
            var callSrv = selectedHopSrv;
            var retObj = ${testService}(callSrv).postSync();
            console.log(retObj)
            var success = retObj.success;
            var results = retObj.output;
            var inputParams = retObj.input;
            var alertClass = '"alert alert-info"';
            var alertMessage = '';
            console.log();
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
            for(index = 0 ; index < results.length ; index++)
            {
              res_literal += "<li>" +   results[index] + '</li>';
            }
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
  if(srvName in srvMap){
    var results = srvMap[srvName]();
    response = {success: results.success, output: results.output,
                input: results.input
               };
  }
  else{
    response = {success: false, output: ['Error on service call!'],
                input: 'Error on service call!'
               }
  }

  return response;
}

var srvMap = {
  'ontology_subclasses_of': test_ontology_subclassesOf,
  'ontology_superclasses_of': test_ontology_superclassesOf,
  //'ontology_is_subsuperclass_of': test_is_subsuperclassOf,
  //'speech_detection_sphinx4': test_speech_detection_sphinx4,
  //'speech_detection_google': test_speech_detection_google,
  //'face_detection': test_face_detection,
  //'qr_detection': test_qr_detection,
  //'text_to_speech': test_text_to_speech
}



function test_ontology_subclassesOf(){
  import service ontology_subclasses_of();
  var query = 'Oven';
  var success = true;
  var response = ontology_subclasses_of({query: query}).postSync();
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
  return {success: success, output: response.results, input: {query: query}};
}

function test_ontology_superclassesOf(){
  import service ontology_superclasses_of();
  var query = 'Oven';
  var response = ontology_superclasses_of({query: query}).postSync();
  console.log(response)
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
  return {success: success, output: response.results, input: {query: query}};
}



function loopInf(){
  setTimeout(function(){
    if( !ros.connected() ){ros.connect(rosbridgeEnv.ip_addr, rosbridgeEnv.port)}
    else{
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
    }
    available_services().post( function( response ){
      platformHopServices = response.services;
      //console.log(response.services)
    })

    loopInf();
  }, pollCycle)
}


