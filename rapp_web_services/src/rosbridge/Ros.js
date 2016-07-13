var Roslib = require('roslib');
var util = require('util');
var ConnectionHandler = require('./ConnectionHandler.js');

/**
 * Rosbridgejs wrapper. Establish connections to ROS through the
 * rosbridge websocket server.
 *
 * TODO: Support passing a custom logger.
 */
function ROS(opts) {
  opts = opts || {};

  this.isConnected = false;
  this.reconnect = opts.reconnect || false;
  this.url = 'ws://localhost:9090';

  this.logger = opts.logger || console;

  this.ros = new Roslib.Ros({
    encoding: 'utf8'
  });

  // Connection Handler
  this.conHandler = new ConnectionHandler(this);

  // Not a service but a server client. Calls to different ROS services can be
  // performed through a signel Service instance.
  this.srvClient = new Roslib.Service({
    ros: this.ros,
    name: '',  // To be specified on service call
    serviceType: ''  // Is this necessary??!!?!?
  });
}


ROS.prototype.callService = function(svcname, reqMsg, onresponse, onerror) {
  onresponse = onresponse || function() {};
  onerror = onerror || function() {};

  var req = new Roslib.ServiceRequest(reqMsg);

  if (! this.isConnected) {
    onerror("Web Service is not connected to ROS!");
  }

  var that = this;

  if (svcname && typeof svcname === 'string') {
    this.srvClient.name = svcname;
    this.srvClient.callService(
      req,
      function(result) {
        //console.log(result);
        onresponse.call(that, result);
      },
      function(err) {
        //console.log(err);
        onerror.call(that, err);
      });
  }
};


/**
 * ROS connection through the rosbridge websocket server.
 *
 */
ROS.prototype.connect = function(opts) {
  opts = opts || {};
  // Only Websocket connection is supported.
  if (typeof opts === 'string' && opts.slice(0, 5) === 'ws://') {
    this.url = opts;
  } else if (opts.host && opts.port) {
    this.url = util.format('ws://%s:%s', opts.host, opts.port);
  }

  this.ros.connect(this.url);
};


/**
 * Disconnect
 */
ROS.prototype.close = function() {
  if (this.isConnected) {
    this.ros.close();
  }
};

module.exports = ROS;
