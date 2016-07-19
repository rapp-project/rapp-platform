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
 * @fileOverview
 *
 * [Available-Services] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */

var path = require('path');

var interfaces = require(path.join(__dirname, 'iface_obj.js'));

/* -- Set timer values for websocket communication to rosbridge -- */
var scanTimer = 2 * 60 * 1000;  // Minutes
var initScanWait = 10 * 1000; // Seconds;
/* --------------------------------------------------------------- */

var __availableServices = [];

onmessage = function(msg) {
  __availableServices.length = 0;
  for (let i in msg.data) {
  // HOP sends a weird array value sometimes !!!
    if (msg.data[i].car !== undefined) {
      continue;
    }
    __availableServices.push(msg.data[i]);
  }
};

/***
 *  Scan services for up-and-running available services.
 *  Scan timer value is used triggers this function invocation.
 */
setTimeout(function getActiveServices() {
  var msg = {
    worker_name: WORKER.name,
    request: "active_services"
  };
  postMessage(msg);

  setTimeout(function() {
    getActiveServices();
  }, scanTimer);

  }, initScanWait
);


/**
 *  [Available-Services], RAPP Platform Front-End Web Service.
 *  Returns a list of currently available RAPP Platform Web Services.
 *
 */
function svcImpl(req, resp, ros) {
  var response = new interfaces.client_res();
  response.services = __availableServices;
  resp.sendJson(response);
}


module.exports = svcImpl;
