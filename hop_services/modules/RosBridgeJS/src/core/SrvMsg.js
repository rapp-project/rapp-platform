/*!
 * @file SrvMsg.js
 * @brief Expose service message objects (srv/ dir).
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

var Srv   = require( __dirname + '/../../srv/Srv.js' );
var Nodes = require( __dirname + '/../../srv/Nodes.js' );
var Param = require( __dirname + '/../../srv/Param.js' );
var Topics= require( __dirname + '/../../srv/Topics.js' );


var GetServices = function(id)  {
  var msg = Srv.GetServices;
  msg.id = id;
  return msg;
};

var GetNodes = function()  {return Nodes.GetNodes;};

var GetParamNames = function()  {return Param.GetParamNames;};

var GetTopics = function()  {return Param.GetParamNames;};

var GetParam = function(paramName){
  var msg = Param.GetParam;
  msg.args.name = paramName;
  return msg;
};


module.exports = {
  GetServices: GetServices,
  GetNodes: GetNodes,
  GetParam: GetParam,
  GetParamNames: GetParamNames,
  GetTopics: GetTopics
};

