var hop = require('hop');


function getToken( req ){
  return req.header['accept-token'];
}


/**
 *  Call the application authentication service to authenticate
 *  client against requested service.
 */
var authRequest = function ( req, svcName, successClb, failClb ){
  // Get token from request object
  service auth_service_access();
  var token = getToken(req);

  if ( ! token ){
    failClb();
    return;
  }

  var options = {
    // service call failure callback
    fail: failClb
  };

  var authSvcFrame = auth_service_access({
    token: token,
    svc_name: svcName
  });

  authSvcFrame.post(function(response){
    if ( response.error ){
      failClb(response.error);
    }
    else{
      successClb(response.username);
    }
  }, options);
};


var responseAuthFailed = function(){
  var options = {
    startLine: "HTTP/1.1 401 Unauthorized"
  };
  var response = hop.HTTPResponseString("Authentication Failure", options);
  return response;
};


function RappAuth(ros) {
  this.rosSvcName = '/rapp/rapp_application_authentication/authenticate_token';
  this.ros = ros;

  this.getToken = function(req) {
    return req.header['accept-token'];
  };
}


// Call the authentication ROS node.
RappAuth.prototype.call = function(req, onSuccess, onFailed) {
  var _this = this;
  var token = this.getToken(req);

  function callback(data){
    var success = data.success;
    var error = data.error;
    var username = data.username;
    if (error){
      onFailed(error);
    }
    else{
      onSuccess(username);
    }
  }

  function onerror(e){
    _this.onFailed(e);
  }

  var rosMsg = {
    token: this.getToken(req)
  };

  this.ros.callService(_this.rosSvcName, rosMsg, {
    success: callback,
    fail: onerror});
};


module.exports.RappAuth = RappAuth;
module.exports.authRequest = authRequest;
module.exports.responseAuthFailed = responseAuthFailed;
