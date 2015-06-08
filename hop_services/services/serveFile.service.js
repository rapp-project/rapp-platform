/**
 * This service is used to serve files under request.
 */

var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
var module_path = rapp_hop_path +  'utilities/js/'

var hop = require ( 'hop' );
/*<Require File Utilities module>*/
var Fs = require( module_path + 'fileUtils.js' );

/*!
 * @brief Service under file requests.
 * @param _filePath Requested file name provided by path.
 *
 * Requested file name might be provided without path in the future. 
 * This means that the user might me authenticated to have access to specific 
 * directories.
 */
service serveFile( _filePath ) 
{  
  var fileURL = Fs.resolvePath( _filePath );
  //var file = serveFile.resource (_filePath);
  console.log ( '[ServeFile Service]: Serving file [%s]', fileURL );
  /* <Returns the requested file (data)> */
  return hop.HTTPResponseFile ( fileURL );
}

