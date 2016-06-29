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


/*!
 * @brief Response object to be passed to the request callback
 * @param hopSendResponse The HOP sendResponse object.
 */
function Response (hopSendResponse) {
  this.send = hopSendResponse;
  this.routes = [];
}


/*!
 * @brief Send an application/json response.
 * @param obj_ The json object to send.
 */
Response.prototype.sendJson = function(obj) {
  this.send(hop.HTTPResponseJson(obj));
};


/*!
 * @brief Send an application/json error response.
 * @param obj_ The json object to send.
 */
Response.prototype.sendError = function(obj) {
  this.send(hop.HTTPResponseJson(obj));
};



/*!
 * @brief Send HTTP 500 Server Error..
 */
Response.prototype.sendServerError = function(msg) {
  msg = msg || "Server Error";
  var options = {
    startLine: "HTTP/1.1 500 Internal Server Error"
  };
  this.send(hop.HTTPResponseString(msg, options));
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
  this.send(hop.HTTPResponseString(msg, options));
};


/*!
 * @brief Request object to pass to the request callback.
 */
function Request(hopReq, hopArgs) {
  this._hopArgs = hopArgs;
  this.body = {};
  this.header = {};
  this.socket = {};

  this.header = hopReq.header || {};
  this.socket = hopReq.socket || {};

  for (var k in hopArgs){
    this.body[k] = hopArgs[k];
  }
}


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
    // Instantioate a new Request object.
    var _req = new Request(this, kwargs);
    /***
     * Asynchronous http response
     */
    return hop.HTTPResponseAsync( function( sendResponse ) {
      // Instantioate a new Response object.
      var _resp = new Response(sendResponse);

      for (var i in that.reqParsers) {
        that.reqParsers[i].call(_req);
      }

      // Authenticate.
      that.auth.call(_req, _resp,
        // On authentication success.
        function(username) {
          //console.log(username);
          _req.username = username;
          // Call registered onRequest callback
          that.onRequest(_req, _resp, that.ros);
        },
        // On authentication failure.
        function(e) {
          // Remove all received files
          for(var k in _req.files){
            for(var i in _req.files[k]){
              Fs.rmFile(_req.files[k][i]);
            }
          }
          console.log(e);
          // Response Authentication Failure
          _resp.sendUnauthorized();
        }
      );

      // Timeout this request. Return to client with Error 500.
      setTimeout(function(){
        // Remove all received files
        for (var k in _req.files) {
          for (var i in _req.files[k]) {
            Fs.rmFile(_req.files[k][i]);
          }
        }
        _resp.sendServerError();
      }, that.timeout);

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
  this.ros = new ROS({
    hostname: ENV.ROSBRIDGE.HOSTNAME,
    port: ENV.ROSBRIDGE.PORT,
    reconnect: true,
    onconnection: function() {
      // .
    }
  });
};



module.exports = WebService;
