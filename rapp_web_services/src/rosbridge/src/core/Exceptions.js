/*!
 *
 * @file Exceptions.js
 * @brief Custom Error Exception container classes.
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

var util = require('util');

function WSError(settings){
  settings = ( settings || {} );

  // Capture the current stacktrace and store it in the property "this.stack"
  Error.captureStackTrace(this, this.constructor);

  // Override the default name property (Error).
  // This is basically zero value-add.
  this.name = this.constructor.name;

  var error = "WebSocket error";
  this.message = (settings.message || error);
  this.details = ( settings.details || '' );
  this.errorCode = ( settings.errorCode || '' );
  this.extendedInfo = ( settings.extendedInfo || '' );
  this.isWebSocketError = true;
}


function RosbridgeError(settings){
  settings = ( settings || {} );

  // Capture the current stacktrace and store it in the property "this.stack"
  Error.captureStackTrace(this, this.constructor);

  // Override the default name property (Error).
  // This is basically zero value-add.
  this.name = this.constructor.name;

  var error = "Rosbridge error";
  this.message = (settings.message || error);
  this.details = ( settings.details || '' );
  this.errorCode = ( settings.errorCode || '' );
  this.extendedInfo = ( settings.extendedInfo || '' );
  this.isWebSocketError = true;
}


util.inherits( WSError, Error );
util.inherits( RosbridgeError, Error );


module.exports = {
  WSError: WSError,
  RosbridgeError: RosbridgeError
}
