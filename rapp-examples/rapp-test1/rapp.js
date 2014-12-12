#!/usr/bin/env node
 
/**
 * This is just a mock-up of the RAPP::API for Javascript
 * It serves to demonstrate the usage of calling a service on the cloud,
 * or calling a core agent method
 */


/**
 * Define the RAPPServices object, which needs a username and password
 */
function RAPPServices ( usr, pwd )
{
    this.cloud_url = "http://api.rapp.cloud";
    this.user = usr;
    this.pwd = pwd;
}

/**
 * Prototype the Functions of RAPPServices.
 */
RAPPServices.prototype = 
{
    // NOTE: As with C++, this is a procedural way of doing things. Eventually, could be object-based
    FooSync: function ( param1 )
    {
        return "bar";
    },
    FooAsync: function ( param1 )
    {
        return "bar";
    }
}

/**
 * Same as before, define the RAPPCore API.
 * The only prerequisite from this approach, 
 * is that we have to version the API carefully, and somehow force updates on the robot if/when the RAPP::API changes.
 * The Javascript executing on robot, would automatically include (by using node.js or require.js) the correct
 * RAPPCore JS file, thus resolving all dependency issues.
 */
function RAPPCore ( )
{
    this.core_url = "127.0.0.1";
}

RAPPCore.prototype =
{
    Foo: function ( param1 )
    {
        return "bar";
    }
}

module.exports.RAPPServices = RAPPServices;
module.exports.RAPPCore = RAPPCore;