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
 *  @author Konstantinos Panayiotou, [klpanagi@gmail.com]
 *
 *  @fileOverview
 *
 *  Illustrates the implementation of a HOP Service, a.k.a
 *  RAPP Platform Web Service.
 *
 */

var util = require('util');

var rosSrvName = '/rapp/dummy';


/**
 *  Template RAPP Platform Web Service
 *
 *  This template web service has two input arguments:
 *    name: A String service input parameter.
 *    number: An integer service input parameter.
 *
 *  This is the callback function that will be executed on request
 *  arrival and after successful authentication (If the web service requires
 *  authentication)

 *  The following arguments are passed to the web service implementation
 *  function:
 *
 *  @param req - The request object. Holds request information like
 *  body values, file uploads paths, request headers, etc.
 *
 *  @param resp - The response object. Holds response methods used to send
 *  responses to the client.
 *
 *  @param ros - Use this object to call ROS Services.
 */
function template_web_svc(req, resp, ros) {
  // Web Service arguments are members of the req.body object
  var name = req.body.name || "Uknown";
  var number = req.body.number || -1;

  // Request headers
  console.log(util.format("Request headers: %s", req.header));
  // The request body. File uploads are excluded.
  console.log(util.format("Request body: %s", req.body));
  // Uploaded files. multipart/form-data file upload post fields.
  console.log(util.format("Uploaded files: %s", req.files));
  // RAPP user username
  console.log(util.format("RAPP user username: %s", req.username));

  var response = {
    msg: util.format('Hello %s #%s', name, number),
    error: ''
  };

  if (name === "Uknown" || number === -1) {
    response.error = 'Missing input arguments';
    resp.sendJson(response);
    return;
  }

  /**
   * ros.callService() method is used to call a ROS Service. This method has
   * three (3) input arguments:
   *
   * ros.callService(rosSrvName, rosMsg,
   *  {success: function(){}, fail: function(){}})
   */

  // ROS Service request message
  var rosMsg = {
    name: name,
    number: number
  };

  function onsuccess(rosResp) {
    resp.sendJson(rosResp);
  }

  function onfailure(err) {
    resp.sendJson({error: err});
  }

  // Call ROS-Service.
  ros.callService(rosSrvName, rosMsg, {success: onsuccess, fail: onfailure});
}


// Export the web service implementation function
module.exports = template_web_svc;
