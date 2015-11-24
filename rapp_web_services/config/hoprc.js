/**
 * HOP authenticated users.
 */

var hop = require( 'hop' );
var user = require( hop.user );
var config = require( hop.config );




/*!
 * @brief RAPP default user. Used as the default user for RAPP developers.
 *
 *  The rapp user have access to all registered HOP services.
 *  Permitted to access directories are:
 *   - /tmp
 *   - ~/
 */
user.add({
  name: "rapp",
  password: "+0c490944da919b09163ce2b72ab2f93a",
  groups: [],
  services: "*",
  directories: ["/tmp", "/home/rapp"]
});


/*!
 * @brief RAPP administrator user. Do not use this unless you know what you are
 *  doing!!
 *
 *   The admin user has global system access rights.
 */
user.add({
  name: "admin",
  password: "+12f253d7a2b993c9926ecb678105a9e3",
  groups: [],
  services: "*",
  directories: "*"
});


/*!
 * @brief The anonynous RAPP user. Used mainly while debugging the RAPP
 *  infrastucture. Do not use this unless you know what you are
 *  doing!!
 *
 *  This user can be used while intefering through the **lo** interface.
 */
user.add({
  name: "anonymous",
  groups: [],
  services: "*",
  directories: ""
});


