/*!
 * @file rosParamClient.js
 * @brief Middleware to retrieve parameters from ROS Parameter Server.
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

var RandomStringGenerator = require( __dirname + '/../utils/randStrGen.js' );
var Exceptions = require( __dirname + '/Exceptions.js' );

var util = require('util');

function ServiceController(args) {
  this.hostname_ = args.hostname;
  this.port_ = args.port;
  this.requests_ = {};
  this.unqIdLength_ = 10;
  this.randStrGen_ = new RandomStringGenerator(this.unqIdLength_);
  var __this = this;

  // Event callback handlers
  this.events_ = {
    onopen: args.onopen || function(){},
    onclose: args.onclose || function(){},
    onerror: args.onerror || function(){}
  };


  // Add a new request.
  this.addRequest = function(msg, callback)
  {
    var reqId = this.randStrGen_.createUnique();
    msg.id = reqId;
    if (this.requests_[ reqId.toString() ] === undefined && this.ws_) {
      this.requests_[reqId.toString()] = callback;
      try{  this.ws_.send(JSON.stringify(msg)); }
      catch(e){
        this.randStrGen_.removeCached(reqId.toString());
        this.events_.onerror();
        this.clearRequest(reqId);
        return false;
      }
    }
  };

  this.clearRequest = function(reqId) {
    if(this.requests_[ reqId.toString() ] !== undefined) {
      delete this.requests_[ reqId.toString() ]; //release this request.
    }
  };

  // Establish connection to the websocket server.
  this.connect = function() {
    if (this.ws_ === undefined) {
      this.ws_ = new WebSocket('ws://' + this.hostname_ + ':' + this.port_);

      // Onopen event handler.
      this.ws_.onopen = function(){
        var msg = util.format(
          'Connection to rosbridge websocket server established: {%s:%s}',
          __this.hostname_, __this.port_);
        console.log(msg);
        __connected = true;
        __this.events_.onopen();
      };

      // Onclose event handler.
      this.ws_.onclose = function(){
        console.log('Connection to rosbridge websocket server closed');
        __this.ws_ = undefined;
        __this.events_.onclose();
      };

      // Onmessage event handler.
      this.ws_.onmessage = function(event){
        var response = JSON.parse(event.value);
        if (__this.requests_[response.id] !== undefined)
        {
          //console.log(response)
          __this.requests_[response.id](response);
          __this.clearRequest(response.id);
        }
      };

      // Onerror event handler.
      this.ws_.onerror = function(e){
        //var errorMsg = util.format("Connection error for [ws://%s:%s] : %s",
          //__this.hostname_, __this.port_, e);
        //console.log(errorMsg);
        //console.log(e)
        __this.events_.onerror();
        __this.ws_.close();
        __this.ws_ = undefined;
      };
    }
  };

  this.connect(this.hostname_, this.port_);


  this.disconnect = function() {
    this.ws_.close();
    this.ws_ = undefined;
  };

  this.registerService = function(msg, callback) {
    if (!callback) {
      console.log("Invoke this method with a valid callback function.");
      return false;
    }
    if (! this.ws_) {
      // Assign the callback to this request.
      this.events_.onerror();
      return false;
    }
    if (this.addRequest(msg, callback)) {
      return true;
    }
    else {
      return true;
    }
  };

}



/**
 * Module exports
 */
module.exports = ServiceController;
