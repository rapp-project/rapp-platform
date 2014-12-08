var fs = require ('fs');
var fsbin = require( "./fsbinary.js" );

//console.log( "fsbin=", fsbin );

import service serveFile (name);

serveFile ("ice.jpg").post(
    function (string)
    {
      console.log ('file downloaded');
      fsbin.writeFileSync ('ice_copy.jpg', string );
    },
    {
      asynchronous: false,  //wait for service to return
      host: "localhost",  //localhost to run locally
      port: 9001,
      //user: "rapp",     //enable these for authentication
      //password: "R@pPpaSS!",
      fail: function( err ) { console.log( " connection refused: ", err ) }
    }
  );
