var fs = require ('fs');
var fsbin = require( "./fsbinary.js" );

console.log( "fsbin=", fsbin );

import service serveFile (name);

serveFile ("t1.mp3").post( function (string) {
	console.log ('file downloaded');
	fsbin.writeFileSync ('t2.mp3', string );
    }, {port: 8888});
