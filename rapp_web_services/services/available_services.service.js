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


/**
 * @file
 *
 * [Available-Services] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var __DEBUG__ = false;

var hop = require('hop');
var path = require('path');

var __configDir = path.join(__dirname,'..', 'config');

var srvEnv = require( path.join(__configDir, 'env', 'hop-services.json') );

var __hopServiceName = 'available_services';
var __hopServiceId = null;
var __availableServices = [];


/* -- Set timer values for websocket communication to rosbridge -- */
var scanTimer = srvEnv[__hopServiceName].scan_time * 60 * 1000;  // Minutes
/* --------------------------------------------------------------- */

var color = {
  error:    String.fromCharCode(0x1B) + '[1;31m',
  success:  String.fromCharCode(0x1B) + '[1;32m',
  ok:       String.fromCharCode(0x1B) + '[34m',
  yellow:   String.fromCharCode(0x1B) + '[33m',
  clear:    String.fromCharCode(0x1B) + '[0m'
};


// Register communication interface with the master-process
register_master_interface();


(function scanServices(){
  __availableServices.length = 0;

  var msg = color.yellow +
    '\n------------------------------------------------------------\n' +
    ' --->   [Scanning Web Services for availability]           ' +
    '\n------------------------------------------------------------\n' +
    color.clear;

  console.log(msg);

  for(var s in srvEnv){
    if(s === __hopServiceName){
      __availableServices.push(s);
      continue;
    }
    var response = undefined;
    var args = {};
    var webService = hop.webService('http://' + hop.hostname + ':' + hop.port +
      '/hop/' + s);
    var srv = webService(args);

    try{
      response = srv.postSync();
    }
    catch(e){
      console.log(color.error + "\n ---> [Failed]: " + s + color.clear +
        "\n");
      continue;
    }
    console.log(color.success + "\n ---> [OK]: " + s + color.clear + "\n");
    __availableServices.push(s);
  }

  console.log(color.ok + "\n <UpRunning Services>" + color.clear);

  for(var i in __availableServices){
    console.log("    *[%s] - %s", i, __availableServices[i]);
  }

  msg = color.yellow +
    '\n------------------------------------------------------------' +
    '\n------------------------------------------------------------\n' +
    color.clear;
  console.log(msg);

  setTimeout(function(){
    scanServices();
  }, scanTimer);
})();


/**
 *  [Available-Services], RAPP Platform Front-End Web Service.
 *  Returns a list of currently available RAPP Platform Web Services.
 *
 *  @function available_services
 *
 *  @param {Object} args - Service input arguments (literal). Empty literal.
 *
 *  @returns {Object} response - JSON HTTPResponse object.
 *  @returns {Array} response.services - List of RAPP Platform available
 *  services.
 *  @returns {String} response.error - Error message string to be filled
 *  when an error has been occured during service call.
 *
 */
service available_services (  )
{
  postMessage( craft_slaveMaster_msg('log', 'client-request') );

   //Async http response
  //-----------------------------------------------------------------
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
      var response = craft_response();
      sendResponse( hop.HTTPResponseJson(response) );
    }, this);
}


function craft_response()
{
  var response = {services: __availableServices, error: ''};
  return response;
}


function craft_error_respose()
{
  var response= {services: [], error: 'RAPP Platform Failure'};
  return response;
}


function register_master_interface()
{
  onexit = function(e){
    console.log("Service [%s] exiting...", hopServiceName);

    var logMsg = "Received termination command. Exiting.";
    postMessage( craft_slaveMaster_msg('log', logMsg) );
  };

  onmessage = function(msg){
    if (__DEBUG__)
    {
      console.log("Service [%s] received message from master process",
        hopServiceName);
      console.log("Msg -->", msg.data);
    }
    /* -------< Add to log file >---------- */
    var logMsg = 'Received message from master process --> [' +
      msg.data + ']';
    postMessage( craft_slaveMaster_msg('log', logMsg) );
    /* ------------------------------------ */

    exec_master_command(msg.data);
  };

  // On initialization inform master and append to log file
  var logMsg = "Initiated worker";
  postMessage( craft_slaveMaster_msg('log', logMsg) );
}


function exec_master_command(msg)
{
  var cmd = msg.cmdId;
  var data = msg.data;
  switch (cmd)
  {
    case 'id':  // Set worker ID
      __hopServiceId = data;
      break;
    default:
      break;
  }
}


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

