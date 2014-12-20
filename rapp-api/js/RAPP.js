#!/usr/bin/env node

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
    /// WARNING: Hardcoding Paths May be BAD Practice - MAYBE, Delegate all Service Calls to a Single Point?
    Service: function ( parameter, callback )
    {
        var request = require('request');        
        request( "http://localhost/service.php", 
                 function ( error, response, body ) 
        {
            if ( !error && response.statusCode == 200)
                callback( body );
            else if ( error )
                console.log ( error );
            else if ( response.statusCode != 200 )
                console.log ( response.statusCode );
        })
    }
}

/// Export
module.exports.RAPPServices = RAPPServices;