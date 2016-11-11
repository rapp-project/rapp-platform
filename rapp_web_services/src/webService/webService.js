var path = require('path');
var fs = require('fs');
var util = require('util');
var hopService = require('hop').Service;
var hop = require('hop');
var Fs = require(path.join(__dirname, '../common/fileUtils.js'));
var auth = require('./auth.js');
var ROS = require(path.join(__dirname, '../rosbridge/src/Rosbridge.js'));
var ENV = require(path.join(__dirname, '../../env.js'));
var bodyParser = require(path.join(__dirname, 'bodyParser.js'));
var fileParser = require(path.join(__dirname, 'fileParser.js'));
var logger = require(path.join(__dirname, '../common/logger.js'));


/*!
 * @brief Response object to be passed to the request callback
 * @param hopSendResponse The HOP sendResponse object.
 */
function Response (hopSendResponse, socket, logger) {
  this.hopSend = hopSendResponse;
  this.socket = socket || {};
  this.routes = [];
  this.logger = logger;
  this.done = false;
}

/*!
 * @brief General Send method
 *
 * @type msg: string || object
 * @param msg: Response Message
 *
 * @type options: object
 * @param options: Response Options used when sending 4xx, 5xx errors
 */
Response.prototype.send = function(msg, options){
  if (this.done) {
    return;
  }
  msg = msg || {};
  options = options || {};
  var sendObj = {};
  if (typeof msg === 'string'){
    sendObj = hop.HTTPResponseString(msg, options);
  }
  else {
    sendObj = hop.HTTPResponseJson(msg, options);
  }
  this.hopSend(sendObj);
  this.logger.log("Response " + this.socket.hostAddress + ':' +
    this.socket.port, msg);
  this.done = true;
};


/*!
 * @brief Send an application/json response.
 * @param obj_ The json object to send.
 */
Response.prototype.sendJson = function(obj) {
  this.send(obj);
};


/*!
 * @brief Send HTTP 500 Server Error..
 */
Response.prototype.sendServerError = function(msg) {
  msg = msg || "Server Error";
  var options = {
    startLine: "HTTP/1.1 500 Internal Server Error"
  };
  this.send(msg, options);
};


/*!
 * @brief Send 401 Unaihorized Client Error. Use for unauthorized access to
 * the server resources.
 */
Response.prototype.sendUnauthorized = function(msg) {
  msg = msg || "Authentication Failure";
  var options = {
    startLine: "HTTP/1.1 401 Unauthorized"
  };
  this.send(msg, options);
};


/*!
 * @brief Request object to pass to the request callback.
 */
function Request(hopReq, hopArgs, logger) {
  this._hopArgs = hopArgs;
  this.body = {};
  this.header = {};
  this.socket = {};

  this.header = hopReq.header || {};
  this.socket = hopReq.socket || {};

  for (var k in hopArgs){
    this.body[k] = hopArgs[k];
  }

  this.logger = logger;
}

Request.prototype.stringify = function(){
  return JSON.stringify({
    body: this.body,
    header: this.header,
    socket: this.socket
  });
};


/*!
 * Web Service implementation. Implemented on top of hop.js. Zero
 * knowledge of the hop.js framework is required.
 *
 * @param {Function} onRequest - Callback function to register for the
 * onrequest event. This function is called on arrival of a new request to
 * the Web Service. Function Arguments:
 *  - {Object} req The request object.
 *  - {Object} resp The response object.
 *  - {Objec} ros A connection to ros.
 *
 *  onRequest(req, resp, ros)
 *
 * @param {Object} options - Options
 * @oaram {Sring} name - The service name.
 * @param {String} urlname - The service urlname. Service name can be different
 * from the urlname.
 * @param {String} options.namespace - Namespace for the urlname to append
 * as a prefix to the service url name. For example, a service with
 * urlname="faca_detection" and namespace="computervision" will be
 * translated to: /computervision/face_detection
 * @param {Boolean} options.anonymous - If true, this service will be
 * anonymous, which means that it will be assigned a random url path.
 * @param {Number} options.timeout - Request timeout value.
 */
function WebService (onRequest, options) {
  var that = this;

  // Hold a ros connection.
  this.ros = null;
  // Hold the onRequest registered callback function
  this.onRequest = onRequest;
  this.options = options;

  // Parameters
  //----------------------------------------------
  this.svc = undefined;
  this.name = options.name;
  this.urlName = options.urlName;
  this.workerName = options.workerName;
  this.anonymous = options.anonymous || false;
  this.namespace = options.namespace || "";
  this.rosSrvName = options.rosSrvName || "";
  this.timeout = options.timeout || 45;
  //----------------------------------------------
  this.logger = new logger(
    {
      debug: ENV.DEBUG,
      logging: ENV.LOGGING,
      logdir: ENV.PATHS.LOG_DIR_SRVS,
      logname: this.name
    }
  );

  this.logger.log(util.format("Logging at %s", this.logger.logpath));


  if (options.ros_bridge || options.auth){
    // Assign a ros-connection to this service instance
    this.connectROS();
  }

  // Apply authentication.
  this.auth = new auth.RappAuth(this.ros);

  // Request parsers
  // ----------------------------------------------
  this.reqParsers = [bodyParser.json()];
  this.reqParsers.push(fileParser());
  if (options.reqParsers) {
    for (var i in options.reqParsers) {
      this.reqParsers.push(options.reqParsers[i]);
    }
  }
  //----------------------------------------------

  /*!
   * @brief The HOP Service implementation. Ready to create with:
   *   new hop.Service(svcImpl)
   */
  this.svcImpl = function(kwargs){
    // Instantiate a new Request object.
    var _req = new Request(this, kwargs);
    for (var i in that.reqParsers) {
      that.reqParsers[i].call(_req);
    }
    // Asynchronous http response
    return hop.HTTPResponseAsync( function( sendResponse ) {
      that.logger.log(
        util.format("Request from %s",
          _req.socket.hostAddress + ':' + _req.socket.port),
        {header: _req.header, body: _req.body, files: _req.files}
      );
      // Instantioate a new Response object.
      var _resp = new Response(sendResponse, _req.socket, that.logger);
      try {
        // ------------ Authenticate ---------------
        that.auth.call(_req, _resp,
          // On authentication success.
          function(username)  {
            _req.username = username;
            // Call registered onRequest callback
            that.onRequest(_req, _resp, that.ros);
          },
          // On authentication failure.
          function(e) {
            that.deleteReqFiles(_req);
            that.logger.log(e);
            // Response Authentication Failure
            _resp.sendUnauthorized();
          }
          );
        // -----------------------------------------

        // Timeout this request. Return to client with Error 500.
        setTimeout(function() {
          // Remove all received files
          for (var k in _req.files) {
            for (var i in _req.files[k]) {
              Fs.rmFile(_req.files[k][i]);
            }
          }
          _resp.sendServerError();
        }, that.timeout);
      }
      catch (e) {
        logger.log(e);
        _resp.sendServerError();
      }

    }, this);

  };

}


/*!
 * @brief Instantiate and register this HOP Web Service.
 */
WebService.prototype.register = function () {
  this.svc = new hopService( this.svcImpl );

  if (! this.anonymous_) {
    // Set HOP Service name
    this.svc.name = (this.namespace) ?
      util.format("%s/%s", this.namespace, this.urlName) : this.urlName;
  }

  // Register service to service handler.
  var msg = {
    request: "svc_registration",
    svc_name: this.name,
    worker_name: this.workerName,
    svc_frame: this.svc,
    svc_path: this.svc.path
  };
  postMessage(msg);

};


/*!
 * @brief Connect this service to ROS through the rosbridge protocol.
 *  Connection over websockets.
 */
WebService.prototype.connectROS = function() {
  var that = this;
  this.ros = new ROS({
    hostname: ENV.ROSBRIDGE.HOSTNAME,
    port: ENV.ROSBRIDGE.PORT,
    reconnect: true,
    onconnection: function() {
      that.logger.log(
        util.format("Established connection to Rosbridge websocket server [%s]",
          ENV.ROSBRIDGE.HOSTNAME + ':' + ENV.ROSBRIDGE.PORT)
      );
    },
    onclose: function() {
      that.logger.log('Connection to rosbridge websocket server closed');
    },
    onerror: function(e) {
      that.logger.log('Rosbridge error: ' + e);
    }
  });
};


WebService.prototype.deleteReqFiles = function (req) {
  // Remove all received files
  for(var k in req.files){
    for(var i in req.files[k]){
      Fs.rmFile(req.files[k][i]);
    }
  }
};


module.exports = WebService;
