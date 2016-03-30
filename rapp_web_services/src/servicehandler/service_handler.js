/*!
 * @file service_handler.js
 * @brief Hop service handler.
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


var path = require('path');
var hop = require('hop');
var util = require('util');

const ENV = require( path.join(__dirname, '../..', 'env.js') );
const PKG_DIR = ENV.PATHS.PKG_DIR;
const INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var Rsg = require ( path.join(INCLUDE_DIR, 'common', 'randStringGen.js') );
var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );


/*!
 * @brief ServiceHandler Prototype.
 * @param {Object} args - Arguments.
 * @param {Object} args.logger - Pass a logger to the Prototype constructor.
 */
function ServiceHandler( args ){
  args = args || {};
  this.logger = {log: undefined, error: undefined};
  this.logger = args.logger;

  this.services = {};
  this.workers = {};
  this.workers.main_thread = {
    services: []
  };
  this.randStrLength = parseInt(args.rand_str_lenth) || 5;
  this.strGen = new Rsg( this.randStrLength );

  /** Load server parameters.
   * ------------------------ */
  this.serverParams = {
    hostname: hop.hostname,
    port: hop.port,
    protocol: (require(hop.config).HTTPSPort) ? "https" : "http"
  };
}


/*!
 * @TODO Enable registration of services under main worker thread.
 */
ServiceHandler.prototype.registerService = function( svcName, frame, path, wName ){
  if( !(svcName && frame && path && wName) ){
    this.logger.error("Service registration failed.");
    return false;
  }

  var svcObj = {
    worker: wName,
    path: path,
    url: this.serviceUrl(path),
    frame: frame
  };

  if( this.workerExists(wName) && (! this.serviceExists(svcName)) ){
    // Append to worker services array
    this.workers[ wName ].services.push(svcName);
    // Append service object to services array.
    this.services[ svcName ] = svcObj;
    this.logger.info(
      util.format("Registered worker service {%s} under worker thread {%s}",
        this.services[ svcName ].url, wName)
    );
    this.logger.info(svcObj);
    return true;
  }
  else{
    this.logger.error(
      util.format("{%s} service registration failed. Either non existed " +
        " worker {%s} , or service with same name already registered", svcName,
        wName)
    );
    return false;
  }
};


/*!
 * @brief Parse worker messages.
 * @param {Object} msg - Communication message
 * @param {Object} msg.data - Message data.
 * @param {String} msg.data.svc_name - Caller service name.
 * @param {String} msg.data.worker_name - Caller worker name.
 * @param {String} msg.data.svc_path - Caller service path (url).
 * @param {Function} msg.data.svc_frame - Caller service frame.
 */
ServiceHandler.prototype.parseWorkerMsg = function( msg ){
  var wName = msg.data.worker_name || "";
  var request = msg.data.request || "";
  var response = {};

  if( (! wName) || (! this.workerExists(wName)) ){
    this.logger.error("Received message from non registered worker");
  }

  switch( request ){
    case "svc_registration":
      // Request to register service
      var svcName = msg.data.svc_name || '';
      var svcFrame = msg.data.svc_frame || undefined;
      var svcPath = msg.data.svc_path || '';

      this.registerService(svcName, svcFrame, svcPath, wName);
      break;

    case "get_svc_url":
      var svcName = msg.data.svc_name || "";
      if( ! ( svcName && serviceExists(svcName) ) ){
        response.error = util.format(
          "Service %s does not exist in registed services", svcName);
        response.svc_name = svcName;
        response.svc_url = '';
      }
      else{
        response.error = '';
        response.svc_url = this.getSvcUrl( svcName );
      }

      this.workers[ wName ].postMessage(response);
      break;

    case "active_services":
      var activeServices = [];
      for ( var k in this.services ){
        activeServices.push(
          {
            name: k,
            url: this.services[ k ].url
          }
        );
      }
      this.workers[ wName ].postMessage(activeServices);
      break;

    default:
      break;
  }
};


/*!
 * @brief Send message to worker thread.
 * @param {String} wName - Worker's name to call.
 * @param {Object} msg - Message object.
 */
ServiceHandler.prototype.callWorker = function( wName, msg ){
  if (this.workerExists(wName)){
    this.workers[ wName ].postMessage(msg);
  }
  else{
    this.logger.info(
      util.format("Attempt to call not existed worder service %s", wName)
    );
  }
};


/*!
 * @brief Kill/Terminate/Close worker service.
 * @param {String} wName - Worker name.
 */
ServiceHandler.prototype.killWorker = function( wName ){
  if ( this.workerExists(wName) ){
    this.workers[ wName ].terminate();
    this.logger.warn("Terminated worker: %s", wName);
  }
};


/*!
 * @brief Creates and instantiates a worker thread.
 * @param {String} wName - Worker name.
 * @param {String} wFile - JS file to feed to worker thread.
 */
ServiceHandler.prototype.createWorker = function( wName, wFile ){
  wName = wName || '';
  wFile = wFile || '';

  // Check if worker with given name already exists in list of registered
  // workers.
  if( this.workerExists(wName) ){
    this.logger.error(util.format("Worker with name {%s} " +
      "exists in registered workers", wName));
    return false;
  }

  var this_ = this;

  try{
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


/*!
 * @brief Register worker service.
 * A worker can hold multiple services;
 * @param {Object} worker - Service information.
 * @param {String} worker.file - Path to worker file.
 * @param {String} worker.name - Service name.
 */
ServiceHandler.prototype.registerWorker = function( worker )
{
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
  if( ! worker.file ){
    this.logger.error("Worker registration failed. Empty worker.file property");
    return false;
  }
  // Check if worker with given name already exists in list of registered
  // workers.
  if( this.workerExists(worker.name) ){
    this.logger.error(util.format("Worker named {%s} already registered"),
      worker.name);
    return false;
  }

  // Instantiate a new worker thread.
  this.createWorker(worker.name, worker.file);
};


/*!
 * @brief Return service url by given service path.
 * @param {String} path - Service's path (e.g. /hop/face_detection).
 */
ServiceHandler.prototype.serviceUrl = function( path ){
  return util.format('%s://%s:%s%s', this.serverParams.protocol,
    this.serverParams.hostname, this.serverParams.port, path);
};


ServiceHandler.prototype.setSvcUrl = function( svcName, path ){
  path = path || '';
  svcName = svcName || '';
  if( ! path ){
    this.logger.error("Non service path provided!");
    return false;
  }
  if( ! svcName ){
    this.logger.error("Non service name provided!");
    return false;
  }
  if( ! serviceExists(svcName) ){
    this.logger.error(util.format("Cannot set %s service url path. " +
      "Service is not registerd!", svcName)
    );
    return false;
  }
  this.services[ svcName ].url = this.serviceUrl( path );
  this.services[ svcName ].path = path;
  return true;
};


ServiceHandler.prototype.getSvcUrl = function( svcName ){
  svcName = svcName || '';
  if( ! svcName ){
    this.logger.error("Non service name provided!");
    return '';
  }
  if( ! serviceExists(svcName) ){
    this.logger.error(util.format("Cannot get %s service url path. " +
      "Service is not registerd!", svcName)
    );
    return '';
  }
  return this.services[ svcName ].url;
};


/*!
 * @brief Returns true if a service has been registered.
 * @param {String} svcName - Service name.
 */
ServiceHandler.prototype.serviceExists = function( svcName ){
  if ( this.services[ svcName ] ) {
    return true;
  }
  else{
    return false;
  }
};


/*!
 * @brief Returns true if a worker has been registered.
 * @param {String} wName - Worker name.
 */
ServiceHandler.prototype.workerExists = function( wName ){
  if ( this.workers[ wName ] ) {
    return true;
  }
  return false;
};



// Export Service Handler as module.
module.exports = ServiceHandler;
