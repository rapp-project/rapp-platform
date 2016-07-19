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


var Bridge = require(__dirname + '/core/bridge.js');

function Rosbridge(options) {
  this.options = options || {};
  this.reconnect = options.reconnect || false;

  this.rosbridge = null;

  this.connect(options);
}


/*!
 *  @brief Get parameter from ROS-Parameter-Server. Service call.
 *
 *  @param paramName Name of the parameter to retrieve from ROS
 *    parameter server.
 *
 *  @param {callback} callback to execute on asynchronous response.
 */
Rosbridge.prototype.getParam = function( paramName, callback, opts ){
  this.rosbridge.getParam(paramName, callback);
};


/*!
 *  @brief Get list of available ROS-Services.
 *
 *  @param {callback} callback to execute on asynchronous response.
 */
Rosbridge.prototype.getServices = function( callback, opts ){
  this.rosbridge.getServices(callback, opts);
};


/*!
 *  @brief Get list of running ROS-Nodes.
 *
 *  @param {callback} callback to execute on asynchronous response.
 */
Rosbridge.prototype.getNodes = function( callback, opts ){
  this.rosbridge.getNodes(callback, opts);
};


/*!
 *  @brief Get list of available ROS-Topics.
 *
 *  @param {callback} callback to execute on asynchronous response.
 */
Rosbridge.prototype.getTopics = function( callback, opts ){
  this.rosbridge.getTopics(callback, opts);
};


/*!
 *  @brief On demand connect to rosbridge websocket server.
 *
 *  @param options Connection options(settings) literal.
 *    { hostname: '', port: '', reconnect: false,
 *      onconnection: function(),
 *      onclose: function(),
 *      onerror: function()
 *    }
 */
Rosbridge.prototype.connect = function(options){
  this.rosbridge = new Bridge(options);
};


/*!
 *  @brief Return true if connection to rosbridge is established.
 */
Rosbridge.prototype.isConnected = function(){
  return this.rosbridge.isActive();
};


/*!
 *  @brief Call a ROS-Service. Service request.
 *
 *  @param srvName ROS-Service name to call.
 *  @param args ROS-Service request arguments.
 *  @callback Callback function to execute on response from service
 *    request call.
 */
Rosbridge.prototype.callService = function( srvName, args, options){
  this.rosbridge.callSrv(srvName, args, options);
};


/*!
 *  @brief Close/Terminate this connection to rosbridge websocket server.
 */
Rosbridge.prototype.closeConnection = function(){
  this.rosbridge.disconnect();
};


module.exports = Rosbridge;
