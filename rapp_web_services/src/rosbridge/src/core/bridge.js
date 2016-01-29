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

var colors = {
  error:    '\033[1;31m',
  success:  '\033[1;32m',
  clear:    '\033[0m'
}


function bridge(options)
{
  options = ( options || {} );
  var reconnect__ = ( options.reconnect || false );
  var pollTimeout__ = 1000;
  var hostname__ = ( options.hostname || 'localhost' );
  var port__ = ( options.port || '9090' );


  var __ServiceController = require( __dirname + '/ServiceController.js' );
  var __SrvMsg = require( __dirname + '/SrvMsg.js' );
  var active__ = false;

  // Init service controller
  var controller__ = new __ServiceController({
    hostname: hostname__,
    port: port__,
    onopen: function(){
      active__ = true;
      // If onconnection callback has been defined on initiation, execute
      if (options.onconnection) { options.onconnection(); }
    },
    onclose: function(){
      active__ = false;
      // If onclose callback has been defined on initiation, execute
      if (options.onclose) { options.onclose(); }
    },
    onerror: function(){
      active__ = false;
      // If onerror callback has been defined on initiation, execute
      if (options.onerror) { options.onerror(); }
    }
  });


  this.getParam = function( paramName, callback, opts ){
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        'Rosbridge connection is not active...';
      //console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);

      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = __SrvMsg.GetParam(paramName);
    controller__.registerService(srvMsg, function(data){
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
      var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        'Rosbridge connection is not active...';
      //console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);

      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = __SrvMsg.GetServices();
    controller__.registerService(srvMsg, function(data){
      if( ! data ) { return; }
      if( data.result ) { callback(data.values.services); }
      // Catch rosbridge_websocket_server error messages
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      }
    });
  };


  this.getNodes = function( callback, opts ){
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        'Rosbridge connection is not active...';
      //console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);

      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = __SrvMsg.GetNodes();
    controller__.registerService(srvMsg, function(data){
      if( ! data ) { return; }
      if( data.result ) { callback(data.values.nodes); }
      // Catch rosbridge_websocket_server error messages
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      }
    });
  };


  this.getTopics = function( callback, opts ){
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        'Rosbridge connection is not active...';
      //console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);

      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    var srvMsg = __SrvMsg.GetTopics();
    controller__.registerService(srvMsg, function(data){
      if( ! data ) { return; }
      if( data.result ) { callback(data.values.names); }
      // Catch rosbridge_websocket_server error messages
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
      }
    });
  };


  this.callSrv = function( srvName, args, opts ){
    opts = opts || {};
    // If rosbridge connection is not estalished, inform and return.
    if(! this.isActive()){
      var errMsg = '[Rosbridge]: Cannot call ros-service. ' +
        'Rosbridge connection is not active...';
      //console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);

      if ( opts.fail ) { opts.fail(errMsg); }
      return false;
    }
    // If an active connection is present, push request to service
    // controller.
    var srvMsg = __SrvMsg.CallSrv(srvName, args);
    controller__.registerService(srvMsg, function(data){
      if( ! data ) { return; }
      if( data.result ) {
        if( opts.success ) { opts.success(data.values); }
      }
      // Catch rosbridge_websocket_server error messages
      else{
        var errMsg = data.values.toString();
        console.log(colors.error + '[Rosbridge]: ' + errMsg + colors.clear);
        if( opts.fail ) { opts.fail(errMsg); }
      }
    });
    return true;
  };


  this.disconnect = function(){
    controller__.disconnect();
  };

  this.isActive = function(){
    return active__;
  };


  var __reconnectController = function(hostname, port){
    console.log('Connecting bridge, {host: %s, port: %s}', hostname__,
      port__);
    controller__.connect(hostname, port);
  };


  var __loop = function(){
    setTimeout( function(){
      if( ! active__ )
      {
        __reconnectController(hostname__, port__);
      }
      __loop();
    }, pollTimeout__);
  };
  if( reconnect__ ) { __loop(); }
}

module.exports = bridge;
