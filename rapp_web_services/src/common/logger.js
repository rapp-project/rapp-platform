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
 * @description Debug to console output
 *
 * @author Konstantinos Panayiotou <klpanagi@gmail.com>
 * @copyright Rapp Project EU 2015
 *
 */

var winston;
try{
  winston = require('winston');
}
catch( e ){
  console.error("Could not load winston node.js: " + e);
}

var path = require('path');


/**
 * @class logger
 *
 * @description Commonly used logger implementation (winston)
 */
var Logger = function(args) {
  args = args || {};
  this.ns = args.ns || '';
  var defaultColors= {
    info: 'cyan', log: '', error: 'red', warn: 'yellow', trace: 'gray',
    debug: 'blue', verbose: '', data: '', help: '', prompt: ''
  };
  this.debug = args.debug || false;
  this.logdir = args.logdir || '/tmp';
  this.ns = args.logname || 'uknown';
  this.logname = this.ns + '-' + Date.now() + '.log';
  this.logpath = path.join(this.logdir, this.logname);
  this.debug = args.debug || false;

  var colors = args.colors || defaultColors;
  try{
    this._logger = new winston.Logger({
      colors: colors
    });

    if (args.logging) {
      this._logger.add(winston.transports.File,{
        level: 'info',
        filename: this.logpath,
        handleExceptions: true,
        json: false,
        timestamp: true
      });
    }

    if (this.debug) {
      this._logger.add(winston.transports.Console, {
        level: 'info',
        prettyPrint: true,
        colorize: true,
        silent: false,
        handleExceptions: true,
        json: false,
        timestamp: true
      });
    }
  }
  catch(e){
    console.log(e);
  }

  this.info = this._logger.info;
};


Logger.prototype.log = function(msgStr, metadata) {
  this._logger.log("info", "[%s] ", this.ns, msgStr,
    metadata !== undefined ? metadata : {});
};

module.exports = Logger;
