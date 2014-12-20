#!/usr/bin/env node

// Import the RAPP JS API
var rapp = require('./RAPP.js');

// Init services
var services = new rapp.RAPPServices( "alex", "qwepoi" );

/** 
 * This is the method that will handle the reply by services.Service
 * Do what you want with it - REMEMBER: Services are Asynchronous!!!
 */
function handler ( data )
{
    console.log ( data );
}

services.Service( "foo", handler );

// TODO: We Need a List of RAPP services