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
      failClb();
    }
    else{
      successClb();
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


module.exports.authRequest = authRequest;
module.exports.responseAuthFailed = responseAuthFailed;
