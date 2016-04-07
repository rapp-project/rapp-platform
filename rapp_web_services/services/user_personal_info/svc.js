/***
 * Copyright 2015 RAPP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Konstantinos Panayiotou
 * Contact: klpanagi@gmail.com
 *
 */


/***
 * @fileOverview
 *
 * [Rapp-Platform-Status] RAPP Platform front-end web service. HTML response.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var hop = require('hop');
var path = require('path');
var util = require('util');

var PKG_DIR = ENV.PATHS.PKG_DIR;
var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var svcUtils = require(path.join(INCLUDE_DIR, 'common', 'svc_utils.js'));

var auth = require(path.join(INCLUDE_DIR, 'common', 'auth.js'));

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.user_personal_info;
var rosSrvName = svcParams.ros_srv_name;
/* ----------------------------------------------------------------------- */

// Initiate communication with rosbridge-websocket-server
var ros = new ROS({hostname: ENV.ROSBRIDGE.HOSTNAME, port: ENV.ROSBRIDGE.PORT,
  reconnect: true, onconnection: function(){
    // .
  }
});


/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */



function svcImpl( kwargs )
{
  var request = this;

  /***
   * Asynchronous http response
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
      auth.authRequest(request, svcParams.name, authSuccess, authFail);

      function authSuccess(user){
        kwargs = kwargs || {};
        var req = new interfaces.client_req();
        var response = new interfaces.client_res();
        var error = '';

        /* Sniff argument values from request body and create client_req object */
        try{
          svcUtils.parseReq(kwargs, req);
        }
        catch(e){
          error = "Service call arguments error";
          response.error = error;
          sendResponse( hop.HTTPResponseJson(response) );
          return;
        }
        /* -------------------------------------------------------------------- */

        // Assign a unique identification key for this service request.
        var unqCallId = randStrGen.createUnique();

        var rosSvcReq = new interfaces.ros_req();
        rosSvcReq.req_cols = [
          'username', 'firstname', 'lastname', 'email',
          'language', 'ontology_alias', 'usrgroup', 'created'];
        rosSvcReq.where_data = [{s: ['username', user]}];


        function callback(data){
          // Remove this call id from random string generator cache.
          randStrGen.removeCached( unqCallId );
          // Craft client response using ros service ws response.
          var response = parseRosbridgeMsg( data );
          // Asynchronous response to client.
          sendResponse( hop.HTTPResponseJson(response) );
        }

        function onerror(e){
          // Remove this call id from random string generator cache.
          randStrGen.removeCached( unqCallId );
          var response = new interfaces.client_res();
          response.error = svcUtils.ERROR_MSG_DEFAULT;
          // Asynchronous response to client.
          sendResponse( hop.HTTPResponseJson(response) );
        }

        ros.callService(rosSrvName, rosSvcReq,
          {success: callback, fail: onerror});

      }

      function authFail(error){
        var response = auth.responseAuthFailed();
        sendResponse(response);
      }

      /***
       *  Timeout this request. Return to client.
       */
      setTimeout(function(){
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
      }, svcParams.timeout);
      /* ----------------------------------------------- */

    }, this);
}



function parseRosbridgeMsg(rosbridge_msg)
{
  var trace = rosbridge_msg.trace;
  var resCols = rosbridge_msg.res_cols;
  var resData = rosbridge_msg.res_data;
  var userInfo = resData.length > 0 ? resData[0].s : [];
  var error = rosbridge_msg.error || '';

  var response = new interfaces.client_res();

  if( error ){
    response.error = error;
    return response;
  }

  for (var ii = 0; ii < resCols.length; ii++){
    response.user_info[resCols[ii]] = userInfo[ii];
  }

  return response;
}


module.exports = svcImpl;
