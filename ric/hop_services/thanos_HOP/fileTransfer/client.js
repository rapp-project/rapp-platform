var fs = require ('fs');
var fsbin = require( "./fsbinary.js" );

console.log( "fsbin=", fsbin );

import service serveFile (name);

serveFile ("ice.jpg").post( function (string) {
	console.log ('file downloaded');
	fsbin.writeFileSync ('ice2.jpg', string );
    }, { asynchronous: false,
		    host: "155.207.19.37",
		    port: 9001,
		    user: "rapp",
		    password: "R@pPpaSS!",
		    fail: function( err ) { console.log( "private, with pwd, connection refused: ", err ) }
		    });
