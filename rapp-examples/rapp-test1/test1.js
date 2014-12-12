#!/usr/bin/env node

/**
 * This is a mock-up of a Javascript RAPP.
 * It relies on Node.js for importing other modules, such as the RAPP API.
 * 
 * NOTE: We will NOT ALLOW devs, to mix and match Javascript and C++, only one language is to be used for a RAPP.
 */

// Import the RAPP JS API
var rapp = require('./rapp.js');

//console.log ( rapp );

var services = new rapp.RAPPServices( "alex", "qwepoi" );

console.log( services.FooSync( "foo" ) );
console.log( services.FooAsync( "foo" ) );

var core = new rapp.RAPPCore();

console.log( core.Foo( "foo" ) );