/**
 * This service is used to serve files under request.
 */

var hop = require ('hop');

service serveFile (name) {
  /* <resource returns the requested file's path> */
  var file = serveFile.resource (name);
  console.log ('Serving file %s', name);
  /* <Returns the requested file (data)> */
  return hop.HTTPResponseFile (file);
}
