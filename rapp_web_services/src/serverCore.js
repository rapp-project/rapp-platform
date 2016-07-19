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
 */

/**
 * @file
 * @description Server Core implementation. Register workers/services,
 * handle communication between web services and worker threads, etc.
 *
 * @author Konstantinos Panayiotou <klpanagi@gmail.com>
 * @copyright Rapp Project EU 2015
 */



var path = require('path');
var hop = require('hop');
var util = require('util');

const ENV = require(path.join(__dirname, '../', 'env.js'));
const PKG_DIR = ENV.PATHS.PKG_DIR;
const INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var Rsg = require (path.join(INCLUDE_DIR, 'common', 'randStringGen.js'));
var Fs = require(path.join(INCLUDE_DIR, 'common', 'fileUtils.js'));


/**
 *  Server Core implementation. Register workers/services, handle communication
 *  between web services and worker threads, etc.
 *
 * @class ServerCore
 *
 * @param {Object} args - Arguments.
 * @param {Object} args.logger - Pass a logger to the Prototype constructor.
 */
function ServerCore(opts) {
  opts = opts || {};
  this.logger = opts.logger || console;

  this.services = {};
  this.workers = {};
  this.workers.main_thread = {
    services: []
  };
  this.randStrLength = parseInt(opts.rand_str_lenth) || 5;
  this.strGen = new Rsg(this.randStrLength);

  /** Load server parameters.
   * ------------------------ */
  this.serverParams = {
    hostname: hop.hostname,
    port: hop.port,
    protocol: (require(hop.config).HTTPSPort) ? "https" : "http"
  };
}


/**
 * Add a custom logger for the Server core to use for logging
 *
 * @function applyLogger
 * @memberOf ServerCore
 */
ServerCore.prototype.applyLogger = function(logger) {
  this.logger = logger || console;
};


/**
 * Register a Web Service.
 *
 * @function registerService
 * @memberOf ServerCore
 *
 * @param {string} svcName - The name of the web service
 * @param {Object} frame - Hop Web service frame.
 * @param {string} path - The Web Service relative url path
 * e.g. /face_detection/detect_faces
 * @param {string} wName - The name of the worker to register this web service
 */
ServerCore.prototype.registerService = function(svcName, frame, path, wName) {
  if (!(svcName && frame && path && wName)) {
    this.logger.error("Service registration failed.");
    return false;
  }

  var svcObj = {
    worker: wName,
    path: path,
    url: this.serviceUrl(path),
    frame: frame
  };

  if( this.workerExists(wName) && (! this.serviceExists(svcName)) ) {
    // Append to worker services array
    this.workers[ wName ].services.push(svcName);
    // Append service object to services array.
    this.services[ svcName ] = svcObj;
    this.logger.log(
      util.format("Registered worker service {%s} under worker thread {%s}",
        this.services[ svcName ].url, wName)
    );
    this.logger.log(svcObj);
    return true;
  } else {
    this.logger.error(
      util.format("{%s} service registration failed. Either non existed " +
        " worker {%s} , or service with same name already registered", svcName,
        wName)
    );
    return false;
  }
};


/**
 * Parse worker messages.
 *
 * @function parseWorkerMsg
 * @memberOf ServerCore
 *
 * @param {Object} msg - Communication message
 * @param {Object} msg.data - Message data.
 * @param {String} msg.data.svc_name - Caller service name.
 * @param {String} msg.data.worker_name - Caller worker name.
 * @param {String} msg.data.svc_path - Caller service path (url).
 * @param {Function} msg.data.svc_frame - Caller service frame.
 */
ServerCore.prototype.parseWorkerMsg = function(msg){
  var wName = msg.data.worker_name || "";
  var request = msg.data.request || "";
  var response = {};

  if( (! wName) || (! this.workerExists(wName)) ){
    this.logger.error("Received message from non registered worker");
  }

  switch (request) {
    case "svc_registration":
      // Request to register service
      let svcName = msg.data.svc_name || '';
      let svcFrame = msg.data.svc_frame || undefined;
      let svcPath = msg.data.svc_path || '';

      this.registerService(svcName, svcFrame, svcPath, wName);
      break;

    case "get_svc_url":
      let svcName = msg.data.svc_name || "";
      if ( ! ( svcName && serviceExists(svcName) ) ) {
        response.error = util.format(
          "Service %s does not exist in registed services", svcName);
        response.svc_name = svcName;
        response.svc_url = '';
      }
      else{
        response.error = '';
        response.svc_url = this.getSvcUrl(svcName);
      }

      this.workers[wName].postMessage(response);
      break;

    case "active_services":
      let activeServices = [];
      for (var k in this.services) {
        let obj = {
          name: k,
          url: this.services[k].url
        };
        activeServices.push(obj);
      }
      this.workers[wName].postMessage(activeServices);
      break;

    default:
      break;
  }
};


/**
 * Send message to worker thread.
 *
 * @function callWorker
 * @memberOf ServerCore
 *
 * @param {String} wName - Worker's name to call.
 * @param {Object} msg - Message object.
 */
ServerCore.prototype.callWorker = function( wName, msg ){
  if (this.workerExists(wName)) {
    this.workers[ wName ].postMessage(msg);
  } else {
    this.logger.log(
      util.format("Attempt to call not existed worder service %s", wName)
    );
  }
};


/**
 * Kill/Terminate/Close worker service.
 *
 * @function killWorker
 * @memberOf ServerCore
 *
 * @param {String} wName - Worker name.
 */
ServerCore.prototype.killWorker = function( wName ){
  if ( this.workerExists(wName) ){
    this.workers[ wName ].terminate();
    this.logger.warn("Terminated worker: %s", wName);
  }
};


/**
 * Creates and instantiates a worker thread.
 *
 * @function createWorker
 * @memberOf ServerCore
 *
 * @param {String} wName - Worker name.
 * @param {String} wFile - JS file to feed to worker thread.
 */
ServerCore.prototype.createWorker = function(wName, wFile) {
  wName = wName || '';
  wFile = wFile || '';

  // Check if worker with given name already exists in list of registered
  // workers.
  if (this.workerExists(wName)) {
    this.logger.error(util.format("Worker with name {%s} " +
      "exists in registered workers", wName));
    return false;
  }

  var this_ = this;

  try {
    this.workers[ wName ] = new Worker ( wFile );
    // Each worker holds his own Array of registered services.
    this.workers[ wName ].services = [];
    //  Register 'this' worker onmessage callback
    this.workers[ wName ].onmessage = function(msg){
      this_.parseWorkerMsg(msg);
    };
    // Register worker onexit event callback.
    this.workers[ wName ].onexit = function(){
      this_.logger.error(util.format("Worker {%s} terminated"));
    };
    // Register worker onerror event callback.
    this.workers[ wName ].onerror = function( e ){
      this_.logger.warn(util.format("Worker send error message -> %s", e));
    };
  }
  catch(e){
    this.logger.error(util.format("Failed to launch worker {%s}: %s",
        wName, e));
    return false;
  }

  return true;
};


/**
 * Register worker service.
 * A worker can handle multiple services;
 *
 * @function registerWorker
 * @memberOf ServerCore
 *
 * @param {Object} worker - Service information.
 * @param {String} worker.file - Path to worker file.
 * @param {String} worker.name - Service name.
 */
ServerCore.prototype.registerWorker = function(worker) {
  worker = worker || {};
  /** Worker registration requires the following properties:
   *  name - worker.name.
   *  file - worker.file.
   */
  if( (! worker) || (! worker.name) ){
    this.logger.error("Could not register worker. " +
      "Not a worker name was provided");
    return false;
  }
  if (! worker.file) {
    this.logger.error("Worker registration failed. Empty worker.file property");
    return false;
  }
  // Check if worker with given name already exists in list of registered
  // workers.
  if (this.workerExists(worker.name)) {
    this.logger.error(util.format("Worker named {%s} already registered"),
      worker.name);
    return false;
  }

  // Instantiate a new worker thread.
  this.createWorker(worker.name, worker.file);
};


/**
 * Create web service full url path by given service path.
 *
 * @function serviceUrl
 * @memberOf ServerCore
 *
 * @param {String} path - Service's path (e.g. /hop/face_detection).
 */
ServerCore.prototype.serviceUrl = function(path) {
  return util.format('%s://%s:%s%s', this.serverParams.protocol,
    this.serverParams.hostname, this.serverParams.port, path);
};


/**
 * Set the url path of given web service..
 *
 * @function setSvcUrl
 * @memberOf ServerCore
 *
 * @param {String} svcName - Service name.
 */
ServerCore.prototype.setSvcUrl = function(svcName, path) {
  path = path || '';
  svcName = svcName || '';
  if (! path) {
    this.logger.error("Non service path provided!");
    return false;
  }
  if (! svcName) {
    this.logger.error("Non service name provided!");
    return false;
  }
  if( ! serviceExists(svcName) ){
    this.logger.error(util.format("Cannot set %s service url path. " +
      "Service is not registerd!", svcName)
    );
    return false;
  }
  this.services[svcName].url = this.serviceUrl(path);
  this.services[svcName].path = path;
  return true;
};


/**
 * Get the url path of given web service..
 *
 * @function getSvcUrl
 * @memberOf ServerCore
 *
 * @param {String} svcName - Service name.
 */
ServerCore.prototype.getSvcUrl = function(svcName) {
  svcName = svcName || '';
  if (! svcName) {
    this.logger.error("Non service name provided!");
    return '';
  }
  if (! serviceExists(svcName)) {
    this.logger.error(util.format("Cannot get %s service url path. " +
      "Service is not registerd!", svcName)
    );
    return '';
  }
  return this.services[ svcName ].url;
};


/**
 * Returns true if a service has been registered.
 *
 * @function serviceExists
 * @memberOf ServerCore
 *
 * @param {String} svcName - Service name.
 */
ServerCore.prototype.serviceExists = function(svcName) {
  if (this.services[svcName]) {
    return true;
  } else {
    return false;
  }
};


/**
 *  Returns true if a worker has been registered.
 *
 * @function workerExists
 * @memberOf ServerCore
 *
 * @param {String} wName - Worker name.
 */
ServerCore.prototype.workerExists = function(wName) {
  if (this.workers[wName]) {
    return true;
  }
  return false;
};

var serverCore = new ServerCore();


// Export the se r.
module.exports = serverCore;
