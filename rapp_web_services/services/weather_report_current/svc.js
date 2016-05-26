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
 * [Weather-Report-Current] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_weather_reporter/current_weather";



/**
 *  [Weather-Report-Current]
 *  Handles requests to weather_report_current RAPP Platform Service
 *
 *  Service Implementation.
 *
 */
function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();
  rosMsg.city = req.body.city;
  rosMsg.weather_reporter = req.body.weather_reporter;
  rosMsg.metric = req.body.metric;

  /* ROS-Service response callback. */
  function callback(data){
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  /* ROS-Service onerror callback. */
  function onerror(e){
    resp.sendServerError();
  }

  /* Call ROS-Service. */
  ros.callService(rosSrvName, rosMsg,
    {success: callback, fail: onerror});
}


/***
 * Craft response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {String} response.error - Error message
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

  response = currentWeather;

  return response;
}


module.exports = svcImpl;
