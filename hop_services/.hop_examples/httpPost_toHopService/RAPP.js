#!/usr/bin/env node

/**
 * Define the RAPPServices object, which needs a username and password
 */
function RAPPServices ( )
{
    this.cloud_url = "http://localhost:9001/hop";
}

/**
 * Prototype the Functions of RAPPServices.
 */
RAPPServices.prototype = 
{
    /// Adder Call
    Adder: function ( x, y, callback )
    {
        var request = require('request');
        request.post({
            headers: {'content-type' : 'application/x-www-form-urlencoded'},
            url: this.cloud_url + "/adder",
            body: "{20,30}"
        },
        function ( error, response, body ) 
        {
            if ( !error && response.statusCode == 200)
                callback( body );
            else if ( error )
                console.log ( "Error: " + error );
            else if ( response.statusCode != 200 )
                console.log ( "Error: " + response.statusCode );
        })
    }
}

/// Export
module.exports.RAPPServices = RAPPServices;
