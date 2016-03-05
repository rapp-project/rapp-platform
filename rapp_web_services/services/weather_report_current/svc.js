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
var svcParams = ENV.SERVICES.weather_report_current;
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

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = svcParams.timeout; // ms
var maxTries = svcParams.retries;
/* ----------------------------------------------------------------------- */


function svcImpl( kwargs )
{
  var req = new interfaces.client_req();
  var response = new interfaces.client_res();
  var error = '';

  var clientHostname = this.socket.hostname;
  var clientIpaddr = this.socket.hostAddr;


  /* ------ Parse arguments ------ */
  kwargs = kwargs || {};
  for( var i in req ){
    req[i] = (kwargs[i] !== undefined) ? kwargs[i] : req[i];
  }
  if( ! req.city ){
    error = 'Empty \"city\" argument';
    response.error = error;
    return hop.HTTPResponseJson(response);
  }

  // Assign a unique identification key for this service request.
  var unqCallId = randStrGen.createUnique();

  /***
   * Asynchronous http response.
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      /***
       *  Status flags.
       *===========================*/
      var respFlag = false;
      var retClientFlag = false;
      var wsError = false;
      var retries = 0;
      /*===========================*/

      var rosSvcReq = new interfaces.ros_req();
      rosSvcReq.city = req.city;
      rosSvcReq.weather_reporter = req.weather_reporter;
      rosSvcReq.metric = req.metric;


      function callback(data){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        //console.log(data);
        // Craft client response using ros service ws response.
        var response = parseRosbridgeMsg( data );
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }

      function onerror(e){
        respFlag = true;
        if( retClientFlag ) { return; }
        // Remove this call id from random string generator cache.
        randStrGen.removeCached( unqCallId );
        // craft error response
        var response = new interfaces.client_res();
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        // Asynchronous response to client.
        sendResponse( hop.HTTPResponseJson(response) );
        retClientFlag = true;
      }


      ros.callService(rosSrvName, rosSvcReq,
        {success: callback, fail: onerror});


      function asyncWrap(){
        setTimeout( function(){

         /***
          * If received message from rosbridge websocket server or an error
          * on websocket connection, stop timeout events.
          */
          if ( respFlag || wsError || retClientFlag ) { return; }

          retries += 1;

          var logMsg = 'Reached rosbridge response timeout' + '---> [' +
            timeout.toString() + '] ms ... Reconnecting to rosbridge.' +
            'Retry-' + retries;

          /***
           * Fail. Did not receive message from rosbridge.
           * Return to client.
           */
          if ( retries >= maxTries ){
            randStrGen.removeCached( unqCallId );

            logMsg = 'Reached max_retries [' + maxTries + ']' +
              ' Could not receive response from rosbridge...';

            var response = new interfaces.client_res();
            response.error = svcUtils.ERROR_MSG_DEFAULT;

            // Asynchronous client response.
            sendResponse( hop.HTTPResponseJson(response));
            retClientFlag = true;
            return;
          }
          /*--------------------------------------------------------*/
          asyncWrap();

        }, timeout);
      }
      asyncWrap();
      /*=====================================================================*/
    }, this );
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
  var error = rosbridge_msg.error;

  if( error ){
    response.error = error;
    return response;
  }

  var currentWeather = new interfaces.current_weather();
  currentWeather.date = rosbridge_msg.date;
  currentWeather.temperature = rosbridge_msg.temperature;
  currentWeather.weather_description = rosbridge_msg.weather_description;
  currentWeather.humidity = rosbridge_msg.humidity;
  currentWeather.visibility = rosbridge_msg.visibility;
  currentWeather.pressure = rosbridge_msg.pressure;
  currentWeather.wind_speed = rosbridge_msg.wind_speed;
  currentWeather.wind_temperature = rosbridge_msg.wind_temperature;
  currentWeather.wind_direction = rosbridge_msg.wind_direction;

  response.weather_current = currentWeather;

  return response;
}


registerSvc(svcImpl, svcParams);
