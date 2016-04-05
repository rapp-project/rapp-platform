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

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.available_services;
var rosSrvName = svcParams.ros_srv_name;

var __availableServices = [];

/* -- Set timer values for websocket communication to rosbridge -- */
var scanTimer = svcParams.scan_time * 60 * 1000;  // Minutes
var initScanWait = svcParams.initial_scan_wait;
/* --------------------------------------------------------------- */

onmessage = function( msg ){
  __availableServices.length = 0;
  for ( var i in msg.data ){
  // HOP sends a weird array value sometimes !!!
    if ( msg.data[i].car !== undefined ){
      continue;
    }
    __availableServices.push(msg.data[i]);
  }
}

/***
 *  Scan services for up-and-running available services.
 *  Scan timer value is used triggers this function invocation.
 */
setTimeout(
  function getActiveServices(){
    var msg = {
      worker_name: WORKER.name,
      request: "active_services"
    };
    postMessage(msg);

    setTimeout(function(){
      getActiveServices();
    }, scanTimer);

  }, initScanWait
);


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
  var response = new interfaces.client_res();
  response.services = __availableServices;
  return hop.HTTPResponseJson(response);
}


module.exports = svcImpl;
