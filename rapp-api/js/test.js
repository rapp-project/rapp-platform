#!/usr/bin/env node

// Import the RAPP JS API & Init the Object
var rapp = require('./RAPP.js');
var services = new rapp.RAPPServices( );

/** 
 * This is the method that will handle the reply by services.Service
 * Do what you want with it - REMEMBER: Services are Asynchronous!!!
 */
function handler ( data )
{
    console.log ( "Reply: " + data );
}

// Call Adder
services.Adder( 5, 6, handler );

// Call the TextFile Sender
//services.SendTextFile( "README.first", handler );

//services.SendImageFile( "picture.jpg", handler );

