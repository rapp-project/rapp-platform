var fs = require ('fs');

/* <Require writeFileSync(PATH,DATA) method> */
var fsbin = require( "./fsbinary.js" );

console.log( "fsbin=", fsbin );

import service serveFile (name);

serveFile ("ice.jpg").post(
  function (data)
  {
    console.log ('file downloaded');
    fsbin.writeFileSync ('ice_copy.jpg', data);
  },
  {
    asynchronous: false,  //wait for service to return
    host: "localhost",  //localhost to run locally
    port: 9001,
    /* <Define the below authentication params to access with server> */    
    //user: "rapp", 
    //password: "R@pPpaSS!",
    fail: function( err ) { console.log( "Connection refused: ", err ) }
  }
);
