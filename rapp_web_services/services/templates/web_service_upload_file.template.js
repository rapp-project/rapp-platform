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
 *  RAPP Platform Web Service, which accepts and handles file uploads.
 *
 */


/**
 *  Template RAPP Platform Web Service
 *
 *  This template web service has one input argument:
 *    file: Uploaded file.
 *
 *  This is the callback function that will be executed on request
 *  arrival and after successful authentication (If the web service requires
 *  authentication)

 *  The following arguments are passed to the web service implementation
 *  function:
 *
 *  @param req - The request object. Holds request information like
 *  body properties, file uploads, request headers, etc.
 *
 *  @param resp - The response object. Holds response methods used to send
 *  responses to the client.
 *
 *  @param ros - Use this object to call ROS Services.
 */
function template_web_svc(req, resp, ros) {
  var response = {
    error: ''
  };

  var fPath;

  try {
    /**
     *  Each req.files property is an array of the web service argument.
     *  This is due to the fact that multiple files can be uploaded using
     *  a single post field
     *
     *  For example, if we expect 2 files to be uploaded in a post field,
     *  named upFiles, you can access each one as below:
     *    var f1Path = req.files.upFiles[0]
     *    var f2Path = req.files.upFiles[1]
     *  Make sure to check on existence of those fields!!
     */
    fPath = req.files.file[0]
  }
  catch(e) {
    response.error('Did not receive any file uploads');
    resp.sendJson(response);
    return;
  }

  console.log("Uploaded file: " + fPath);
  resp.sendJson(response);
}


// Export the web service implementation function
module.exports = template_web_svc;
