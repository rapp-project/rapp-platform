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
 * [Weather-Report-Forecast] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

var rosSrvName = "/rapp/rapp_weather_reporter/weather_forecast";



/**
 *  [Weather-Report-Forecast]
 *  Handles requests to weather_report_forecast RAPP Platform Service
 *
 *  Service Implementation.
 *
 *
 */
function svcImpl ( req, resp, ros )
{
  var rosMsg = new interfaces.ros_req();

  rosMsg.city = req.body.city;
  rosMsg.weather_reporter = req.body.weather_reporter;
  rosMsg.metric = req.body.metric;


  /***
   * ROS-Service response callback.
   */
  function callback(data){
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg( data );
    resp.sendJson(response);
  }

  /***
   * ROS-Service onerror callback.
   */
  function onerror(e){
    resp.sendServerError();
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg,
    {success: callback, fail: onerror});

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


module.exports = svcImpl;
