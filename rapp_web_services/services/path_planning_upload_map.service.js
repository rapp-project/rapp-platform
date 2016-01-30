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
 * [Path-planning-upload-map] RAPP Platform front-end web service.
 *
 *  @author Wojciech Dudek
 *  @mail wojciechsbox@gmail.com
 *  @copyright Rapp Project EU 2015
 */



var __DEBUG__ = false;

var hop = require('hop');
var path = require('path');

var __includeDir = path.join(__dirname, '..', 'modules');
var __configDir = path.join(__dirname, '..', 'config');

var Fs = require( path.join(__includeDir, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(__includeDir, 'common',
    'randStringGen.js') );

var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );

var srvEnv = require( path.join(__configDir, 'env', 'hop-services.json') );
var pathsEnv = require( path.join(__configDir, 'env', 'paths.json') );

/* ------------< Load and set global configuration parameters >-------------*/
var __hopServiceName = 'path_planning_upload_map';
var __hopServiceId = null;
var __servicesCacheDir = Fs.resolvePath( pathsEnv.cache_dir_services );
var __serverCacheDir = Fs.resolvePath( pathsEnv.cache_dir_server );
/* ----------------------------------------------------------------------- */

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */

/* ------< Set timer values for websocket communication to rosbridge> ----- */
var timeout = srvEnv[__hopServiceName].timeout; // ms
var maxTries = srvEnv[__hopServiceName].retries;
/* ----------------------------------------------------------------------- */

var colors = {
  error:    String.fromCharCode(0x1B) + '[1;31m',
  success:  String.fromCharCode(0x1B) + '[1;32m',
  ok:       String.fromCharCode(0x1B) + '[34m',
  yellow:   String.fromCharCode(0x1B) + '[33m',
  clear:    String.fromCharCode(0x1B) + '[0m'
};

/**
 *  [Path-planning-upload-map] RAPP Platform front-end web service.
 *  <p> Serves requests for path_planning_upload_map on given input: .png file, .yaml file, user name, map name.</p>
 *
 *  @function path_planning_upload_map
 *
 *  @param {Object} args - Service input arguments (object literal).
 *  @param {String} args.PNGfile_uri - System uri path of transfered (client)
 *    file, as declared in multipart/form-data post field. The PNGfile_uri is
 *    handled and forwared to this service, as input argument,
 *    by the HOP front-end server.
 *    Clients are responsible to declare this field in the multipart/form-data
 *    post field.
 *
 *  @param {String} args.YAMLfile_uri - System uri path of transfered (client)
 *    file, as declared in multipart/form-data post field. The YAMLfile_uri is
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
service path_planning_upload_map({PNGfile_uri: '', YAMLfile_uri: '', user_name: '', map_name: ''})
{

  /***
   *  For security reasons, if PNGfile_uri or YAMLfile_uri is not defined under the
   *  server_cache_dir do not operate. HOP server stores the files under the
   *  __serverCacheDir directory.
   */
  if( PNGfile_uri.indexOf(__serverCacheDir) === -1 )
  {
    var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
        " Abortion for security reasons.";
    postMessage( craft_slaveMaster_msg('log', errorMsg) );
    console.log(colors.error + '[Face-Detection]: ' + errorMsg + colors.clear);

    var response = {success: false, error: errorMsg};

    return hop.HTTPResponseJson(response);
  }
  if( YAMLfile_uri.indexOf(__serverCacheDir) === -1 )
  {
    var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
        " Abortion for security reasons.";
    postMessage( craft_slaveMaster_msg('log', errorMsg) );
    console.log(colors.error + '[Face-Detection]: ' + errorMsg + colors.clear);

    var response = {success: false, error: errorMsg};

    return hop.HTTPResponseJson(response);
  }

  var cpPNGFilePath = "/home/rapp/rapp_platform_files/maps/"+user_name+"/"+map_name+".png";
  var cpYAMLFilePath = "/home/rapp/rapp_platform_files/maps/"+user_name+"/"+map_name+".yaml";
  //
  // create user directory if one does not exist.
  //
  if(!Fs.isDirectory("/home/rapp/rapp_platform_files/maps/"+user_name))
      Fs.createDir("/home/rapp/rapp_platform_files/maps/"+user_name);
  //
  // coppy .png and .yaml files from the server_cache_dir
  //
  Fs.renameFile(PNGfile_uri, cpPNGFilePath);
  Fs.renameFile(YAMLfile_uri, cpYAMLFilePath);

  var response = {success: true, error: ''};
 return hop.HTTPResponseJson(response);
}

function craft_slaveMaster_msg(msgId, msg)
{
  var _msg = {
    name: __hopServiceName,
    id:   __hopServiceId,
    msgId: msgId,
    data: msg
  };
  return _msg;
}
