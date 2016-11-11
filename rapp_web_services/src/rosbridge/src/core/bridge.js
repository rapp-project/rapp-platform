/*!
 * @file bridge.js
 *
 *
 *  Authors: Konstantinos Panayiotou
 *  Contact: klpanagi@gmail.com
 *
 */

var util = require('util');
var Exceptions = require( __dirname + '/Exceptions.js' );
var WSError= Exceptions.WSError;
var RosbridgeError= Exceptions.RosbridgeError;
var ServiceController = require( __dirname + '/ServiceController.js' );
var SrvMsg = require( __dirname + '/SrvMsg.js' );


function Bridge(options) {
  options = ( options || {} );
  this.connectionInfo = {
    reconnect: options.reconnect || false, // Do not reconnect by default
    host: options.hostname || 'localhost', // Connect to localhost by default
    port: options.port || '9090'
  };
  var reconnect__ = ( options.reconnect || false );
  var pollTimeout__ = 3000;


  this.active = false;

  var _that = this;

  // Init service controller
  this.controller = new ServiceController({
    hostname: _that.connectionInfo.host,
    port: _that.connectionInfo.port,
    onopen: function(){
      _that.active = true;
      // If onconnection callback has been defined on initiation, execute
      if (options.onconnection) {
        options.onconnection();
      }
    },
    onclose: function(){
      _that.active = false;
      // If onclose callback has been defined on initiation, execute
      if (options.onclose) {
        options.onclose();
      }
    },
    onerror: function(){
      active = false;
      // If onerror callback has been defined on initiation, execute
      if (options.onerror) {
        options.onerror();
      }
    }
  });


  this.getParam = function( paramName, callback, opts ){
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      //var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        //'Rosbridge connection is not active...';
      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = SrvMsg.GetParam(paramName);

    this.controller.registerService(srvMsg, function(data){
      if( ! data ) { return; }
      if( data.result ) { callback(data.values.value); }
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      }
    });
  };


  this.getServices = function( callback, opts ){
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      //var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        //'Rosbridge connection is not active...';
      //console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = SrvMsg.GetServices();
    this.controller.registerService(srvMsg, function(data){
      if (! data) { return; }
      if (data.result) { callback(data.values.services); }
      // Catch rosbridge_websocket_server error messages
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      }
    });
  };


  this.getNodes = function(callback, opts) {
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      //var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        //'Rosbridge connection is not active...';
      //console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = SrvMsg.GetNodes();
    this.controller.registerService(srvMsg, function(data){
      if( ! data ) { return; }
      if( data.result ) { callback(data.values.nodes); }
      // Catch rosbridge_websocket_server error messages
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      }
    });
  };


  this.getTopics = function(callback, opts) {
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = SrvMsg.GetTopics();
    this.controller.registerService(srvMsg, function(data){
      if( ! data ) { return; }
      if( data.result ) { callback(data.values.names); }
      // Catch rosbridge_websocket_server error messages
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      }
    });
  };


  var __reconnectController = function(hostname, port) {
    //console.log('Connecting bridge, {host: %s, port: %s}',
      //this.hostname, this.port);
    _that.controller.connect();
  };

  var __loop = function(obj) {
    setTimeout( function(){
      if (! _that.isActive()) {
        __reconnectController(_that.hostname, _that.port);
      }
      __loop();
    }, pollTimeout__);
  };

  if (reconnect__) {
    __loop(this);
  }
}


/*!
 * @brief Disconnect client.
 */
Bridge.prototype.disconnect = function () {
  this.controller.disconnect();
};


/*!
 * @brief Check if connection is active.
 */
Bridge.prototype.isActive = function () {
  return this.active;
};


Bridge.prototype.callSrv = function (srvName, args, opts) {
  opts = opts || {};
  // If rosbridge connection is not estalished, inform and return.
  if (! this.isActive()) {
    if ( opts.fail ) { opts.fail(errMsg); }
    return false;
  }
  // If an active connection is present, push request to service
  // controller.
  var srvMsg = SrvMsg.CallSrv(srvName, args);
  this.controller.registerService(srvMsg, function(data){
    if( ! data ) { return; }
    if( data.result ) {
      if( opts.success ) { opts.success(data.values); }
    }
    // Catch rosbridge_websocket_server error messages
    else{
      var errMsg = data.values.toString();
      if( opts.fail ) { opts.fail(errMsg); }
    }
  });
  return true;
};


module.exports = Bridge;
