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


/**
 * @class logger
 *
 * @description Commonly used logger implementation (winston)
 */
var logger = function(args) {
  args = args || {};
  this.ns = args.ns || '';
  var file = args.file || false;
  var defaultColors= {
    info: 'cyan', log: '', error: 'red', warn: 'yellow', trace: 'gray',
    debug: 'blue', verbose: '', data: '', help: '', prompt: ''
  };

  var colors = args.colors || defaultColors;
  try{
    this.logger_ = new winston.Logger({
      colors: colors
    });

    this.logger_.add(winston.transports.Console, {
      prettyPrint: true,
      colorize: true,
      silent: false,
      handleExceptions: true,
      json: false,
      timestamp: true
    });

    if (file) {
      this.logger_.add(winston.transports.File,{
        level: 'log',
        filename: file,
        handleExceptions: true,
        json: false,
        colorize: false
      });
    }
    if (args.debug) {
      this.logger_.add(winston.transports.Console, {
        level: 'debug',
        prettyPrint: true,
        colorize: true,
        silent: false,
        handleExceptions: true,
        json: false,
        timestamp: false
      });
    }
  }
  catch( e ){
    console.log(e);
  }
};

logger.prototype.info = function( msg ){
  if( typeof msg === String ){
    this.logger_.log("info", "[%s] %s", this.ns, msg);
  }
  else{
    this.logger_.log("info", "[%s] ", this.ns, msg);
  }
};

logger.prototype.log = function( msg ){
  if( typeof msg === String ){
    this.logger_.log("debug", "[%s] %s", this.ns, msg);
  }
  else{
    this.logger_.log("debug", "[%s] ", this.ns, msg);
  }
};

logger.prototype.warn = function( msg ){
  if( typeof msg === String ){
    this.logger_.log("warn", "[%s] %s", this.ns, msg);
  }
  else{
    this.logger_.log("warn", "[%s] ", this.ns, msg);
  }
};

logger.prototype.error = function( msg ){
  if( typeof msg === String ){
    this.logger_.log("error", "[%s] %s", this.ns, msg);
  }
  else{
    this.logger_.log("error", "[%s] ", this.ns, msg);
  }
};

logger.prototype.debug = function( msg ){
  this.logger_("debug", "[%s] %s", this.ns, msg);
};

logger.prototype.verbose = function( msg ){
  this.logger_.log("verbose", "[%s] %s", this.ns, msg);
};

logger.prototype.trace = function( msg ){
  this.logger_.log("trace", "[%s] %s", this.ns, msg);
};

module.exports = logger;
