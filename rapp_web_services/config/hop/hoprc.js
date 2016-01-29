var hop = require( 'hop' );
var user = require( hop.user );
var config = require( hop.config );
var fs = require('fs');
var path = require('path');

var rappPlatformDir = path.join(__dirname, '../../..');

/*************************************************************************
 *                      HOP authenticated users.
 *************************************************************************/


var rappAccessDirs = [
  path.join(rappPlatformDir, "rapp_web_services/gui/css"),
  path.join(rappPlatformDir, "rapp_web_services/gui/js"),
  path.join(rappPlatformDir, "rapp_web_services/gui/img"),
  path.join(rappPlatformDir, "rapp_web_services/tests")
];


/*
 * Default RAPP User.
 */
user.add({
  name: "rapp",
  password: "+0c490944da919b09163ce2b72ab2f93a",
  groups: [],
  services: "*", // Allow for all services
  directories: "*" // Restrict directories to /tmp.
});


/*
 * Anonymous user. Only allowed for anonymous local requests.
 */
user.add({
  name: "anonymous",
  groups: [],
  services: "*",
  directories: "" // Do not grant directory access.
});

