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


/**
 * @fileOverview
 *
 * [News-Explore] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

const rosSrvName = "/rapp/rapp_news_explorer/fetch_news";


/**
 *  [News-Explore]
 *  Handles requests to news_explore RAPP Platform Service
 *
 *  Service Implementation.
 *
 */
function svcImpl(req, resp, ros) {
  var rosMsg = new interfaces.ros_req();
  rosMsg.newsEngine = req.body.news_explore;
  rosMsg.keywords = req.body.keywords;
  rosMsg.excludeTitles = req.body.exclude_titles;
  rosMsg.regionEdition = req.body.region;
  rosMsg.topic = req.body.topic;
  rosMsg.storyNum = req.body.num_news;

  // ROS-Service response callback.
  function callback(data) {
    // Parse rosbridge message and craft client response
    var response = parseRosbridgeMsg(data);
    resp.sendJson(response);
  }

  // ROS-Service onerror callback.
  function onerror(e) {
    var response = new interfaces.client_res();
    response.error = e;
    resp.sendJson(response);
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg, {success: callback, fail: onerror});
}


/***
 * Crafts response object.
 *
 *  @param {Object} rosbridge_msg - Return message from rosbridge
 *
 *  @returns {Object} response - Response Object.
 *  @returns {String} response.error - Error message
 */
function parseRosbridgeMsg(rosbridge_msg) {
  //var success = rosbridge_msg.status;
  const error = rosbridge_msg.error;
  const newsStories = rosbridge_msg.stories;

  var response = new interfaces.client_res();

  if (error) {
    response.error = error;
    return response;
  }

  response.news_stories = newsStories;

  return response;
}


module.exports = svcImpl;
