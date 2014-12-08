/*This example shows how to call a HOP service.
*/

import service svc1(n);
var n=1;
var tak= svc1(n).post(function(result){return result},
		  { asynchronous: false,    //if true then service call will be executed asynchronously, program execution will not wait for return. For example the console.log(tak) will print 1 as the service value will not have returned
		    host: "localhost",
		    port: 9001,
		   // user: "rapp",         //user and password are used for authentication
		   // password: "R@pPpaSS!",
		    fail: function( err ) { console.log( " connection refused: ", err ) }
		    } );
console.log(tak);

