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
 * [Available-Services] RAPP Platform front-end web service.
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

var interfaces = require( path.join(__dirname, 'interfaces.json') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.available_services;
var SERVICE_NAME = svcParams.name;
var rosSrvName = svcParams.ros_srv_name;


var __availableServices = [];

/* -- Set timer values for websocket communication to rosbridge -- */
var scanTimer = svcParams.scan_time * 60 * 1000;  // Minutes
var initScanWait = svcParams.initial_scan_wait;
/* --------------------------------------------------------------- */

var colors = {
  error:    String.fromCharCode(0x1B) + '[1;31m',
  success:  String.fromCharCode(0x1B) + '[1;32m',
  ok:       String.fromCharCode(0x1B) + '[34m',
  yellow:   String.fromCharCode(0x1B) + '[33m',
  clear:    String.fromCharCode(0x1B) + '[0m'
};


/***
 *  Scan services for up-and-running available services.
 *  Scan timer value is used triggers this function invocation.
 */
setTimeout(
function scanServices(){
  __availableServices.length = 0;

  var msg = colors.yellow +
    '\n------------------------------------------------------------\n' +
    ' --->   [Scanning Web Services for availability]           ' +
    '\n------------------------------------------------------------\n' +
    colors.clear;

  console.log(msg);

  for(var s in ENV.SERVICES){
    if(s === SERVICE_NAME){
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
      console.log(colors.error + "\n ---> [Failed]: " + s + colors.clear +
        "\n");
      continue;
    }
    console.log(colors.success + "\n ---> [OK]: " + s + colors.clear + "\n");
    __availableServices.push(s);
  }

  console.log(colors.ok + "\n <UpRunning Services>" + colors.clear);

  for(var i in __availableServices){
    console.log("    *[%s] - %s", i, __availableServices[i]);
  }

  msg = colors.yellow +
    '\n------------------------------------------------------------' +
    '\n------------------------------------------------------------\n' +
    colors.clear;
  console.log(msg);

  setTimeout(function(){
    scanServices();
  }, scanTimer);
}, initScanWait);


/**
 *  [Available-Services], RAPP Platform Front-End Web Service.
 *  Returns a list of currently available RAPP Platform Web Services.
 *
 *  @function available_services
 *
 *  @param {Object} args - Service input arguments (literal). Empty literal.
 *
 *  @returns {Object} response - JSON HTTPResponse object.
 *    Asynchronous HTTP Response.
 *  @returns {Array} response.services - List of RAPP Platform available
 *  services.
 *  @returns {String} response.error - Error message string to be filled
 *  when an error has been occured during service call.
 *
 */
function svcImpl ( kwargs )
{
  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
      var response = interfaces.client_response;
      response.services = __availableServices;
      sendResponse( hop.HTTPResponseJson(response) );
    }, this);
}


function service_url(srvName){
  return 'http://' + hop.hostname + ':' + hop.port + '/hop/' + srvName;
}


registerSvc(svcImpl, svcParams);
