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

var RandomStringGenerator = require( '../RandomStrGenerator/randStringGen.js' );


/**!
 * @brief Returns parameter request msg.
 * @param paramName The name of the parameter.
 */
function ParamRequest(paramName, reqId)
{
  return {op: 'call_service',
          service: '/rosapi/get_param',
          args: {name: paramName},
          id: reqId
        };
};


/**
 * *** Include and use the random string generator to craft unique
 *     request caller id and assign each callback to the valid caller.
 * *** Maybe develop a seperate Service Controller. ;)
 */


function RosParamClient(args)
{
  this.hostname_ = args.hostname || 'localhost';
  this.port_ = args.port || '9090';
  this.srvROS_ = '/rosapi/get_param';
  this.requests_ = {};
  this.unqIdLength_ = 10;
  this.randStrGen_ = new RandomStringGenerator(this.unqIdLength);
  var __this = this;

  try{
    this.ws_ = new WebSocket('ws://' + this.hostname_ + ':' + this.port_);
    this.ws_.onopen = function(){
      //console.log('Connection to rosbridge established. Caller id -- %s',id);
    }
    this.ws_.onclose = function(){
      //console.log('Connection to rosbridge closed. Caller id -- %s', id);
    }
    this.ws_.onmessage = function(event){
      var response = JSON.parse(event.value);
      if (__this.requests_[response.id] != undefined)
      {
        __this.requests_[response.id](response.values.value);
        __this.clearRequest(response.id);
      }
    }
  }
  catch(e){
    console.log('Failure on websocket initiation');
    return;
  }

  this.addRequest = function(paramName, callback)
  {
    var reqId = this.randStrGen_.createUnique();
    if( this.requests_[ reqId.toString() ] === undefined )
    {
      this.requests_[reqId.toString()] = callback;
      var msg = new ParamRequest(paramName, reqId);
      this.ws_.send(JSON.stringify(msg));
    }
  }

  this.clearRequest = function(reqId)
  {
    if( this.requests_[ reqId.toString() ] !== undefined )
    {
      this.randStrGen_.removeCached(reqId.toString());
      delete this.requests_[ reqId.toString() ]; //release this request.
    }
  }

}


/**!
 * @brief Retrieve requested parameter value from ROS Parameter Server
 * @TODO
 */
RosParamClient.prototype.getParam = function(paramName, callback)
{
  if (!callback)
  {
    console.log("Invoke this method with a valid callback function.");
    return;
  }
  // Assign the callback to this request.
  this.addRequest(paramName, callback);
}


/**
 * Module exports
 */
module.exports = RosParamClient;
