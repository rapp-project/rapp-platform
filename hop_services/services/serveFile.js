/**
 * This service is used to serve files under request.
 */

var user = process.env.LOGNAME;
var module_path = '../utilities/js/'

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
 *
 * NOT OPERATIONAL!!!
 */
service serveFile( {file: ''} )
{
  var filePath = Fs.resolvePath( file );

  /**
   *
   * TODO Check if is authorized user space and then return.
   */

  //var file = serveFile.resource (_filePath);
  console.log ( '[ServeFile Service]: Serving file [%s]', filePath );
  /* <Returns the requested file (data)> */
  return hop.HTTPResponseFile ( filePath );
}

