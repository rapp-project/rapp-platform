var hop = require ('hop');

service serveFile (name) {
  var file = serveFile.resource (name);
  console.log ('serving file %s', file);
  /* <Returns the requested file (data)> */
  return hop.HTTPResponseFile (file);
}
