var hop = require( 'hop' );
var user = require( hop.user );
var config = require( hop.config );


/*************************************************************************
 *                      HOP authenticated users.
 *************************************************************************/

/*
 * Default RAPP User.
 */
user.add({
  name: "rapp",
  password: "+0c490944da919b09163ce2b72ab2f93a",
  groups: [],
  services: "*", // Allow for all services
  directories: ["/tmp"] // Restrict directories to /tmp.
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

