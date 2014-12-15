/**
 * This service is used to serve files under request.
 */

var hop = require ('hop');

/*!
 * @brief Service under file requests.
 * @param name Requested file name provided only by relative path. 
 */
service serveFile_relative (name) {
  /* <resource returns the requested file's (Absolute) path> */
  var file = serveFile_relative.resource (name);
  console.log ('Serving file %s', name);
  /* <Returns the requested file (data)> */
  return hop.HTTPResponseFile (file);
}

/*!
 * @brief Service under file requests.
 * @param name Requested file name provided only by absolute path. 
 */
//service serveFile_absolute (name) {
  //console.log ('Serving file %s', name);
  //[> <Returns the requested file (data)> <]
  //return hop.HTTPResponseFile (file);
//}

