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

/**
 * @module
 * @description Server-side worker thread utility functions.
 */


var path = require('path');
var util = require('util');

var webService = require(path.join(__dirname, '../webService/webService.js'));


/**
 * @function setWorkerName
 * @description Sets worker's name in global scope object.
 *
 * @param wName - Worker's name to assign;
 */
var setWorkerName = function(wName) {
  if( ! wName ){
    throw new Error("Invalid set of Arguments. Missing wName!");
  }
  global.WORKER = global.WORKER || {name: wName, services: []};
};


/**
 * @function launchSvcAll
 * @description Launch all services registered to the worker thread calling
 * this method
 */
var launchSvcAll = function(){
  var workerSvc = ENV.WORKERS[ WORKER.name ].services;
  var workerDir = path.dirname(path.join(
    ENV.PATHS.PKG_DIR, ENV.WORKERS[ WORKER.name ].path));

  for( var i in workerSvc ){
    var svcParams = ENV.SERVICES[ workerSvc[i] ];
    if( svcParams.launch ){
      var srcPath = '';

      if( svcParams.path ){
        srcPath = path.join(
          ENV.PATHS.SERVICES_DIR, svcParams.path);
      }
      else{
        srcPath = path.join(
          ENV.PATHS.SERVICES_DIR,
          svcParams.name, 'svc.js'
        );
      }

      var svcImpl = require(srcPath);

      var options = {
        name:       svcParams.name,
        urlName:    svcParams.url_name,
        workerName: WORKER.name,
        anonymous:  svcParams.anonymous,
        namespace:  svcParams.namespace,
        timeout:    svcParams.timeout,
        rosSrvName: svcParams.ros_srv_name || "",
        ros_bridge: svcParams.ros_bridge || false,
        auth:       svcParams.authentication || false
      };

      var svc = new webService(svcImpl, options);
      // Register this service to the service handler
      svc.register();
    }
  }
};


// Global worker thread space hack!!
global.ENV = require( path.join(__dirname, '../..', 'env.js') );


exports.setWorkerName = setWorkerName;
exports.launchSvcAll = launchSvcAll;
