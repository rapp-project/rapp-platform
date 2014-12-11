/**
 * Client side example. Call a hop serveFile service in order to 
 * request a file.
 */ 

/* 
 * <Use fsbin instead of fs to transfer binary files> 
 * //var fs = require ('fs');
*/

/* <Require writeFileSync(PATH,DATA) method> */
var fsbin = require( "./fsbinary.js" );
import service serveFile (name);

console.log( "Loading custom binary file streaming method @", fsbin );

serveFile ("ice.jpg").post(
  /* This is a nested function */
  function (data) /* <data == return value of serveFile service> */
  {
    console.log ('file downloaded');
    fsbin.writeFileSync ('ice_copy.jpg', data);
  },
  {
    asynchronous: false,  //wait for service to return
    host: "localhost",  //localhost to run locally
    port: 9001,
    /* <Define the below authentication params in order to access a server> */    
    //user: "rapp", 
    //password: "R@pPpaSS!",
    fail: function( err ) { console.log( "Connection refused: ", err ) }
  }
);
