var hop = require( 'hop' );
var user = require( hop.user );
var config = require( hop.config );


/**
 * HOP authenticated users.
 */
user.add({
  name: "rapp",
  password: "+0c490944da919b09163ce2b72ab2f93a",
  groups: [],
  services: "*",
  directories: ["/tmp", "/home/rapp"]
});


user.add({
  name: "admin",
  password: "+12f253d7a2b993c9926ecb678105a9e3",
  groups: [],
  services: "*",
  directories: "*"
});


user.add({
  name: "anonymous",
  groups: [],
  services: "*",
  directories: ""
});


