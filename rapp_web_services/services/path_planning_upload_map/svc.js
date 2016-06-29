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
 * [Path-planning-upload-map] RAPP Platform web service.
 *
 *  @author Wojciech Dudek
 *  @mail wojciechsbox@gmail.com
 *  @copyright Rapp Project EU 2015
 */


var path = require('path');
var util = require('util');


var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var Fs = require(path.join(INCLUDE_DIR, 'common', 'fileUtils.js'));

var interfaces = require(path.join(__dirname, 'iface_obj.js'));



/**
 *  [Path-planning-upload-map] RAPP Platform web service.
 *  Serves requests for path_planning_upload_map on given input: .png file, .yaml file, user name, map name.</p>
 */
function svcImpl (req, resp, ros) {
  var response = new interfaces.client_res();

  if (! req.files.png_file) {
    response.error = 'No map image file received';
    resp.sendJson(response);
    return;
  }
  if (! req.files.yaml_file) {
    response.error = 'No map image file received';
    resp.sendJson(response);
    return;
  }
  if (! req.body.map_name) {
    response.error = 'Not a map_name argument provided.';
    resp.sendJson(response);
    return;
  }

  if( (typeof req.body.map_name !== "string")){
    response.error = 'map_name must be of type {string}';
    resp.sendJson(response);
    return;
  }

  var cpPNGFile = "~/rapp_platform_files/maps/" + req.username + "/" +
    req.body.map_name + ".png";
  var cpYAMLFile = "~/rapp_platform_files/maps/" + req.username + "/" +
    req.body.map_name + ".yaml";


  // create user directory if one does not exist.
  if (!Fs.isDirectory("~/rapp_platform_files/maps/" + req.username)) {
    Fs.createDirRecur("~/rapp_platform_files/maps/" + req.username);
  }

  // coppy .png and .yaml files from the server_cache_dir
  if (! Fs.renameFile(req.files.png_file[0], cpPNGFile)) {
    response.error = "Failed to upload map png to RAPP Platform.";
    response.success = false;
  }
  if (! Fs.renameFile(req.files.yaml_file[0], cpYAMLFile)) {
    response.error = "Failed to upload map yaml to RAPP Platform.";
    response.success = false;
  }
  else {
    response.success = true;
  }

  resp.sendJson(response);
}


module.exports = svcImpl;
