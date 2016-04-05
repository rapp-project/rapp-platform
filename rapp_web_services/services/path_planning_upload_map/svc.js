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


var hop = require('hop');
var path = require('path');
var util = require('util');


var PKG_DIR = ENV.PATHS.PKG_DIR;
var INCLUDE_DIR = ENV.PATHS.INCLUDE_DIR;

var svcUtils = require(path.join(INCLUDE_DIR, 'common',
    'svc_utils.js'));

var Fs = require( path.join(INCLUDE_DIR, 'common', 'fileUtils.js') );

var RandStringGen = require ( path.join(INCLUDE_DIR, 'common',
    'randStringGen.js') );

var interfaces = require( path.join(__dirname, 'iface_obj.js') );

/* ------------< Load parameters >-------------*/
var svcParams = ENV.SERVICES.path_planning_upload_map;

var SERVICES_CACHE_DIR = ENV.PATHS.SERVICES_CACHE_DIR;
var SERVER_CACHE_DIR = ENV.PATHS.SERVER_CACHE_DIR;
/* ----------------------------------------------------------------------- */

/*----------------< Random String Generator configurations >---------------*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/* ----------------------------------------------------------------------- */



/**
 *  [Path-planning-upload-map] RAPP Platform front-end web service.
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
function svcImpl ( kwargs )
{
  /***
   * Asynchronous http response
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

      if( ! req.png_file ){
        error = 'No map image file received';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( ! req.yaml_file ){
        error = 'No map image file received';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( ! req.user ){
        error = 'Provide username argument (user).';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( ! req.map_name ){
        error = 'Not a map_name argument provided.';
        response.error = error;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }

      /***
       *  For security reasons, if file_uri is not defined under the
       *  server_cache_dir do not operate. HOP server stores the files under the
       *  SERVER_CACHE_DIR directory.
       */
      if( req.png_file.indexOf(SERVER_CACHE_DIR) === -1 )
      {
        var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
          " Abortion for security reasons.";
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      if( req.yaml_file.indexOf(SERVER_CACHE_DIR) === -1 )
      {
        var errorMsg = "Service invocation error. Invalid {file_uri} field!" +
          " Abortion for security reasons.";
        response.error = svcUtils.ERROR_MSG_DEFAULT;
        sendResponse( hop.HTTPResponseJson(response) );
        return;
      }
      /* ----------------------------------------------------------------------- */

      var cpPNGFile = "~/rapp_platform_files/maps/"+req.user+"/"+req.map_name+".png";
      var cpYAMLFile = "~/rapp_platform_files/maps/"+req.user+"/"+req.map_name+".yaml";

      var unqCallId = randStrGen.createUnique();

      // create user directory if one does not exist.
      if(!Fs.isDirectory("~/rapp_platform_files/maps/"+req.user)){
        Fs.createDirRecur("~/rapp_platform_files/maps/"+req.user);
      }

      // coppy .png and .yaml files from the server_cache_dir
      if ( ! Fs.renameFile(req.png_file, cpPNGFile) ){
        response.error = "Failed to upload map png to RAPP Platform.";
        response.success = false;
      }
      if ( ! Fs.renameFile(req.yaml_file, cpYAMLFile) ){
        response.error = "Failed to upload map yaml to RAPP Platform.";
        response.success = false;
      }
      else{
        response.success = true;
      }

      Fs.rmFile(req.png_file);
      Fs.rmFile(req.yaml_file);
      randStrGen.removeCached(unqCallId);

      sendResponse( hop.HTTPResponseJson(response) );

    }, this);
}


module.exports = svcImpl;
