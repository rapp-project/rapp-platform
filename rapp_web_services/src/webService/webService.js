var path = require('path');
var fs = require('fs');
var util = require('util');
var hopService = require('hop').Service;
var hop = require('hop');
var auth = require('./auth.js');
var ROS = require(path.join(__dirname, '../rosbridge/src/Rosbridge.js'));
var ENV = require(path.join(__dirname, '../../env.js'));
var bodyParser = require(path.join(__dirname, 'bodyParser.js'));
var fileParser = require(path.join(__dirname, 'fileParser.js'));



function Response (hopSendResponse) {
  this.send = hopSendResponse;
  this.routes
}


Response.prototype.sendJson = function(obj_){
  this.send(hop.HTTPResponseJson(obj_));
}

Response.prototype.sendError= function(obj_){
  this.send(hop.HTTPResponseJson(obj_));
}



/*!
 * @brief Returns Server Error response object.
 */
Response.prototype.sendServerError= function(msg){
  var _msg = msg || "Server Error";
  var options = {
    startLine: "HTTP/1.1 500 Internal Server Error"
  };
  this.send(hop.HTTPResponseString(_msg, options));
};




/**
 * @param onRequest {Function} - Callback function on request arrival.
 *
 *    function onRequest(req, ros, sendResponse)
 *
 */
function WebService (onRequest, options) {
  // Will be instantiated in register function
  var that = this;

  this.ros;
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
  this.timeout = options.timeout || 30;
  //----------------------------------------------

  if (options.ros_bridge || options.auth){
    // Assign a ros-connection to this service instance
    this.connectROS();
  }


  // Create a RappAuth instance.
  if (options.auth){
    this.auth = new auth.RappAuth(this.ros);
  }

  // Request parsers
  //----------------------------------------------
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
    var _req = getReqObj(this);
    appendBodyToRequest(_req, kwargs);

    for (var i in that.reqParsers) {
      that.reqParsers[i].parse(_req);
    }


    /***
     * Asynchronous http response
     */
    return hop.HTTPResponseAsync( function( sendResponse ) {
      //var rosBridge = {call: function(msg, onsuccess, onerror){
         //Call ROS-Service.
        //console.log('Calling ROS')
        //that.ros.callService(that.rosSrvName, msg,
          //{success: onsuccess, fail: onerror});
        //}
      //}
      var _resp = new Response(sendResponse);

      if (! that.auth){
        // Axreiasto!!!!
        _req.username = req.username;
        // If authentication for this service is disabled
        that.onRequest(_req, _resp, that.ros);
        return;
      }
      that.auth.call(_req, _resp,
        function(username) {
          //console.log(username);
          _req.username = username;
          // Call registered onRequest callback
          that.onRequest(_req, _resp, that.ros);
        },
        function(e) {
          // Remove all received files
          for (var i in _req.files){
            fs.exists(_req.files[i], function(exists){
              if(exists){
                fs.unlink(_req.files[i]);
              }
            });
          }
          console.log(e);
          // Response Authentication Failure
          response = that.responseAuthFailed();
          sendResponse(response);
        }
      );


      /***
       *  Timeout this request. Return to client.
       */
      setTimeout(function(){
        _resp.sendServerError();
      }, that.timeout);
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



/*!
 * @brief Instantiate and register this Web Service.
 */
WebService.prototype.register = function ()
{
  this.svc = new hopService( this.svcImpl );

  if( ! this.anonymous_ ){
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
WebService.prototype.connectROS = function(){
  this.ros = new ROS({
    hostname: ENV.ROSBRIDGE.HOSTNAME,
    port: ENV.ROSBRIDGE.PORT,
    reconnect: true,
    onconnection: function(){
      // .
    }
  });
}



/*!
 * @brief Returns a hop.HTTPResponseJson object.
 */
WebService.prototype.responseJson = function(obj){
  return hop.HTTPResponseJson(obj);
};





/*!
 * @brief Returns Unauthorized Error response object.
 */
WebService.prototype.responseAuthFailed = function(msg){
  var options = {
    startLine: "HTTP/1.1 401 Unauthorized"
  };
  var response = hop.HTTPResponseString("Authentication Failure", options);
  return response;
};


module.exports = WebService;
