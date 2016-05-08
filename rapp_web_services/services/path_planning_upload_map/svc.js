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

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );



/**
 *  [Path-planning-upload-map] RAPP Platform web service.
 *  <p> Serves requests for path_planning_upload_map on given input: .png file, .yaml file, user name, map name.</p>
 *
 *  @function path_planning_upload_map
 *
 *  @param {Object} args - Service input arguments (object literal).
 *  @param {String} args.png_file - System uri path of transfered (client)
 *    file, as declared in multipart/form-data post field. The png_file is
 *    handled and forwared to this service, as input argument,
 *    by the HOP front-end server.
 *    Clients are responsible to declare this field in the multipart/form-data
 *    post field.
 *
 *  @param {String} args.yaml_file - System uri path of transfered (client)
 *    file, as declared in multipart/form-data post field. The yaml_file is
 *    handled and forwared to this service, as input argument,
 *    by the HOP front-end server.
 *    Clients are responsible to declare this field in the multipart/form-data
 *    post field.
 *
 *  @returns {Object} response - JSON HTTPResponse Object.
 *    Asynchronous HTTP Response.
 *  @returns {bool} response.success - Status of performed operations.
 *  @returns {String} response.error - Error message string to be filled
 *    when an error has been occured during service call.
 */
function svcImpl ( req, resp, ros )
{
  var response = new interfaces.client_res();

  if( ! req.files.png_file[0] ){
    response.error = 'No map image file received';
    resp.sendJson(response);
    return;
  }
  if( ! req.files.yaml_file[0] ){
    response.error = 'No map image file received';
    resp.sendJson(response);
    return;
  }
  if( ! req.body.map_name ){
    response.error = 'Not a map_name argument provided.';
    resp.sendJson(response);
    return;
  }

  var cpPNGFile = "~/rapp_platform_files/maps/" + req.username + "/" +
    req.map_name + ".png";
  var cpYAMLFile = "~/rapp_platform_files/maps/" + req.username + "/" +
    req.map_name + ".yaml";


  // create user directory if one does not exist.
  if(!Fs.isDirectory("~/rapp_platform_files/maps/" + req.username)){
    Fs.createDirRecur("~/rapp_platform_files/maps/" + req.username);
  }

  // coppy .png and .yaml files from the server_cache_dir
  if ( ! Fs.renameFile(req.files.png_file[0], cpPNGFile) ){
    response.error = "Failed to upload map png to RAPP Platform.";
    response.success = false;
  }
  if ( ! Fs.renameFile(req.files.yaml_file[0], cpYAMLFile) ){
    response.error = "Failed to upload map yaml to RAPP Platform.";
    response.success = false;
  }
  else{
    response.success = true;
  }

  Fs.rmFile(req.files.png_file[0]);
  Fs.rmFile(req.files.yaml_file[0]);

  resp.sendJson(response);
}


module.exports = svcImpl;
