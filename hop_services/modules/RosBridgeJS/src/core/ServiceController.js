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

//
var RandomStringGenerator = require( __dirname + '/../utils/randStrGen.js' );
var Exceptions = require( __dirname + '/Exceptions.js' );


function ServiceController(args)
{
  this.hostName_ = args.hostName || 'localhost';
  this.port_ = args.port || '9090';
  this.requests_ = {};
  this.unqIdLength_ = 10;
  this.randStrGen_ = new RandomStringGenerator(this.unqIdLength_);
  var __this = this;
  this.events_ = {
    onopen: args.onopen || function(){},
    onclose: args.onclose || function(){},
    onerror: args.onerror || function(){}
  }


  this.addRequest = function(msg, callback)
  {
    var reqId = this.randStrGen_.createUnique();
    msg.id = reqId;
    if( this.requests_[ reqId.toString() ] === undefined && this.ws_ )
    {
      try{  this.ws_.send(JSON.stringify(msg)); }
      catch(e){
        //throw Exceptions.WebSocketError(this.hostName_, this.port_);
        console.log( "\033[0;31m" +
          Exceptions.WebSocketError(this.hostName_, this.port_) + "\033[0m"
        );
        this.randStrGen_.removeCached(reqId.toString());
        this.events_.onerror();
        return;
      }
    }
    this.requests_[reqId.toString()] = callback;
  }

  this.clearRequest = function(reqId)
  {
    if( this.requests_[ reqId.toString() ] !== undefined )
    {
      delete this.requests_[ reqId.toString() ]; //release this request.
    }
  }

  this.connect = function(hostname, port)
  {
    if( this.ws_ == undefined )
    {
      try{
        this.ws_ = new WebSocket('ws://' + this.hostName_ + ':' + this.port_);
        this.ws_.onopen = function(){
          console.log('Connection to rosbridge established');
          __connected = true;
          __this.events_.onopen();
        }
        this.ws_.onclose = function(){
          console.log('Connection to rosbridge closed');
          __this.ws_ = undefined;
          __this.events_.onclose();
        }
        this.ws_.onmessage = function(event){
          var response = JSON.parse(event.value);
          if (__this.requests_[response.id] != undefined)
          {
            __this.requests_[response.id](response);
            __this.clearRequest(response.id);
          }
        }
      }
      catch(e){
        console.log( "\033[0;31m" +
          Exceptions.WebSocketError(this.hostName_, this.port_) + "\033[0m"
          );
        this.events_.onerror();
        //throw Exceptions.WebSocketError(this.hostName_, this.port_);
      }
    }
  }
  this.connect(this.hostName_, this.port_);
}

/**!
 * @brief Retrieve requested parameter value from ROS Parameter Server
 * @TODO
 */
ServiceController.prototype.appendSrv = function(msg, callback)
{
  if (!callback)
  {
    console.log("Invoke this method with a valid callback function.");
    return;
  }
  if( !this.ws_ ){
    // Assign the callback to this request.
    console.log( "\033[0;31m" +
      Exceptions.WebSocketError(this.hostName_, this.port_) + "\033[0m"
      );
    this.events_.onerror();
    callback(undefined);
    return;
  }
  this.addRequest(msg, callback);
}



/**
 * Module exports
 */
module.exports = ServiceController;
