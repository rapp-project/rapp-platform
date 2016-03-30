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
 * [Cognitive-get-history] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 *
 *  Basic Components (modules)
 *
 *      hop = require('hop').
 *        Import and use hopjs functionalities.
 *        For more information visit:
 *            https://github.com/manuel-serrano/hop
 *
 *      RosBridgeJS.js
 *        Use this module to achieve communication with ROS-framework.
 *        This module integrates a service controller to connect to
 *        the rosbridge-websocket-server.
 *        For more information on rosbridge-websocket-server visit:
 *            http://wiki.ros.org/rosbridge_suite
 *
 *        For more information on the RosBridgeJS module visit:
 *            https://github.com/klpanagi/RosBridgeJS
 *
 *      RandStrGenerator.js
 *        Random string generator class to generate cached unique
 *        identity keys. Used to generate a unique id for each client
 *        service request.
 *
 */

var hop = require('hop');
var path = require('path');
var util = require('util');

var PKG_DIR = ENV.PATHS.PKG_DIR;
var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var svcUtils = require(path.join(INCLUDE_DIR, 'common',
    'svc_utils.js'));

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var ROS = require( path.join(INCLUDE_DIR, 'rosbridge', 'src',
    'Rosbridge.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.weather_report_forecast;
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
  /***
   * Asynchronous http response.
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
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

      if( ! req.city ){
        error = 'Empty \"city\" argument';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }

      // Assign a unique identification key for this service request.
      var unqCallId = randStrGen.createUnique();

      var rosSvcReq = new interfaces.ros_req();
      rosSvcReq.city = req.city;
      rosSvcReq.weather_reporter = req.weather_reporter;
      rosSvcReq.metric = req.metric;


      function callback(data){
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = parseRosbridgeMsg( data );
        sendResponse( hop.HTTPResponseJson(response) );
      }

      function onerror(e){
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // craft error response
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
      }

      ros.callService(rosSrvName, rosSvcReq,
        {success: callback, fail: onerror});

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




/***
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *  @returns {Object} response - Response Object.
 *
 */
function parseRosbridgeMsg(rosbridge_msg)
{
  var response = new interfaces.client_res();
  var forecast = rosbridge_msg.forecast;

  var error = rosbridge_msg.error;

  if( error ){
    response.error = error;
    return response;
  }

  for (var i in forecast){
    var forecastEntry = new interfaces.forecast_entry();
    forecastEntry.high_temp = forecast[i].high_temperature;
    forecastEntry.low_temp = forecast[i].low_temperature;
    forecastEntry.description = forecast[i].description;
    forecastEntry.date = forecast[i].date;
    response.forecast.push(forecastEntry);
  }

  return response;
}


registerSvc(svcImpl, svcParams);
