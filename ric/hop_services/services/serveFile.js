/**
 * This service is used to serve files under request.
 */

var hop = require ('hop');
var fs = require('fs');
/*!
 * @brief Service under file requests.
 * @param name Requested file name provided only by relative path. 
 */
service serveFile_relative (name) {
  if(fs.exists(name)){
    /* <resource returns the requested file's (Absolute) path> */
    var file = serveFile_relative.resource (name);
    console.log ('Serving file %s', name);
    /* <Returns the requested file (data)> */
    return hop.HTTPResponseFile (file);
  }
}

/*!
 * @brief Service under file requests.
 * @param name Requested file name provided only by absolute path. 
 */
service serveFile_absolute (name) {
  if(fs.existsSync(name)){
    console.log ('\033[01;32mServing file %s', name);
    /* <Returns the requested file (data)> */
    return hop.HTTPResponseFile (name);
  }
}

