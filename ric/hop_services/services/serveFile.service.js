/**
 * This service is used to serve files under request.
 */
var user = "klpanagi";
var rapp_hop_path = "/home/" + user + 
  "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";

var hop = require ( 'hop' );
/*<Require File Utilities module>*/
var Fs = require( /*rapp_hop_path + */"../utilities/./fileUtils.js" );

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
  var path = Fs.resolvePath( _filePath );
  //var file = serveFile.resource (_filePath);
  console.log ( '\033[01;33m[ServeFile Service]\033[0;0m Serving file \033[0;31m%s\033[0;0m', _filePath );
  /* <Returns the requested file (data)> */
  return hop.HTTPResponseFile ( _filePath );
}

