/*!
 * @file getServicesTest.js
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

function bridge(hostName, port)
{
  var __ServiceController = require( __dirname + '/ServiceController.js' );
  var __SrvMsg = require( __dirname + '/SrvMsg.js' );
  var active__ = false;
  var controller__ = new __ServiceController({
    hostName: hostName,
    port: port,
    onopen: function(){active__ = true;},
    onclose: function(){active_ = false;},
    onerror: function(){active__ = false;}
  });

  this.getParam = function(paramName, callback){
    var srvMsg = __SrvMsg.GetParam(paramName);
    controller__.appendSrv(srvMsg, function(data){
      var response = (data == undefined) ? [] : data.values.value;
      callback(response);
    });
  };

  this.getServices = function(callback){
    var srvMsg = __SrvMsg.GetServices();
    controller__.appendSrv(srvMsg, function(data){
      var response = ( ! data) ? [] : data.values.services;
      callback(response);
    });
  };

  this.getNodes = function(callback){
    var srvMsg = __SrvMsg.GetNodes();
    controller__.appendSrv(srvMsg, function(data){
      var response = ( ! data ) ? [] : data.values.nodes;
      callback(response);
    });
  };

  this.getTopics = function(callback){
    var srvMsg = __SrvMsg.GetTopics();
    controller__.appendSrv(srvMsg, function(data){
      var response = ( ! data ) ? [] : data.values.names;
      callback(response);
    });
  };

  this.initServiceCotroller = function(hostName, port)
    {controller__.connect(hostName, port);}

  this.connected = function()
    {return active__;}
}

module.exports = bridge;
