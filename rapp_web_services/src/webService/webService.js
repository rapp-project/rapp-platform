var path = require('path');
var util = require('util');
var hopService = require('hop').Service;
var hop = require('hop');
var auth = require('./auth.js');
var ROS = require(path.join(__dirname, '../rosbridge/src/Rosbridge.js'));
var ENV = require(path.join(__dirname, '../../env.js'));
var bodyParser = require(path.join(__dirname, 'bodyParser.js'));
var fileParser = require(path.join(__dirname, 'fileParser.js'));


/**
 * @param onRequest {Function} - Callback function on request arrival.
 *
 *    function onRequest(req, ros, sendResponse)
 *
 * @param rosOnMessage {function} - Callback function on ros-msg arrival.
 *
 *    function rosOnMessage(rosmsg, sendResponse)
 */
function WebService(onRequest, options)
{
  // Will be instantiated in register function
  var that = this;

  // Assign a ros-connection to this service instance
  this.ros = new ROS({
    hostname: ENV.ROSBRIDGE.HOSTNAME,
    port: ENV.ROSBRIDGE.PORT,
    reconnect: true,
    onconnection: function(){
      // .
    }
  });

  this.onRequest = onRequest;
  this.options = options;

  this.svc = undefined;
  this.name = options.name;
  this.urlName = options.urlName;
  this.workerName = options.workerName;
  this.anonymous = options.anonymous || false;
  this.namespace = options.namespace || "";

  this.auth = new auth.RappAuth(this.ros);

  // Request parsers
  this.reqParsers = [bodyParser.json()];
  this.reqParsers.push(fileParser());
  if (options.reqParsers) {
    for (var i in options.reqParsers) {
      this.reqParsers.push(options.reqParsers[i]);
    }
  }
  //----------------------------------------------

  this.svcImpl = function(kwargs){
    var _req = getReqObj(this);
    appendBodyToRequest(_req, kwargs);

    for (var i in that.reqParsers) {
      that.reqParsers[i].parse(_req);
    }
    console.log(_req);

    /***
     * Asynchronous http response
     */
    return hop.HTTPResponseAsync( function( sendResponse ) {
      that.auth.call(_req,
        function(username) {
          console.log(username);
          _req.username = username;
          that.onRequest(_req, that.ros, sendResponse);
        },
        function(e) {
          console.log(e);
          var response = auth.responseAuthFailed();
          sendResponse(response);
        }
      );


      /***
       *  Timeout this request. Return to client.
       */
      setTimeout(function(){
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
      }, 40000);
      /* ----------------------------------------------- */


    }, this);

  };


  /**
   *  @brief Extracts req information from a hop-request object and return
   *  the request object.
   *
   *  @param hopReq - A Hop Request object. The this object of a Hop Service.
   *
   *  @returns {Object} - Request (req) object.
   */
  var getReqObj = function (hopReq){
    var _req = {};
    for( var k in hopReq ) {
      //console.log( k, "=", hopReq[ k ] );
      _req[k] = hopReq[k];
    }
    // Remove Hop Service Object existence in request object
    delete _req.service;
    return _req;
  };


  var appendBodyToRequest = function(req, kwargs) {
    req.body = req.body || {};
    for (var k in kwargs){
      req.body[k] = kwargs[k];
    }
  };
}


WebService.prototype.register = function ()
{
  this.svc = new hopService( this.svcImpl );

  if( ! this.anonymous_ ){
    // Set HOP Service name
    this.svc.name = (this.namespace) ?
      util.format("%s/%s", this.namespace, this.urlName) :
      this.urlName;
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



WebService.prototype.responseJson = function(obj){
  return hop.HTTPResponseJson(obj);
};


WebService.prototype.responseServerError = function(msg){
  var _msg = msg || "Server Error";
  var options = {
    startLine: "HTTP/1.1 500 Internal Server Error"
  };
  return hop.HTTPResponseString(_msg, options);

};


WebService.prototype.responseAuthFailed = function(msg){
  var _msg = msg || "Server Error";
  var options = {
    startLine: "HTTP/1.1 500 Internal Server Error"
  };
  return hop.HTTPResponseString(_msg, options);
};

module.exports = WebService;
