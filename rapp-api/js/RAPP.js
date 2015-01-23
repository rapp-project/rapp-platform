#!/usr/bin/env node

/**
 * Define the RAPPServices object, which needs a username and password
 */
function RAPPServices ( )
{
    this.cloud_url = "http://localhost:8080";
}

/**
 * Prototype the Functions of RAPPServices.
 */
RAPPServices.prototype = 
{
    /// Adder Call
    Adder: function ( x, y, callback )
    {
        var rapp = this;
        var request = require('request');
        request.post({
            headers: {'content-type' : 'application/x-www-form-urlencoded'},
            url: rapp.cloud_url + "",
            body: "adder({x:"+x+",y:" + y + "})</EOF!>"
        },
        function ( error, response, body ) 
        {
            if ( !error && response.statusCode == 200)
                callback( body );
            else if ( error )
                console.log (  error );
            else if ( response.statusCode != 200 )
                console.log ( "Error: " + response.statusCode );
        })
    },
    
    /// Send a Text File to Server
    SendTextFile: function ( filename, callback )
    {
        var request = require('request');
        var fs = require('fs');
        var rapp = this;
        
        fs.readFile( filename, 'utf8', function( err, data) {
            if ( err ) throw err;
            
            var post = data + "</EOF!>";
            
            request.post({
                headers: { 'content-type' : 'text/plain; charset=utf9'},
                url: rapp.cloud_url + "",
                body: post
            },
            function ( error, response, body ) 
            {
                if ( !error && response.statusCode == 200)
                    callback( body );
                else if ( error )
                    console.log (  error );
                else if ( response.statusCode != 200 )
                    console.log ( response.statusCode );
            });
        });
    },
    
    /// Send an Image File to Server
    SendImageFile: function ( filename, callback )
    {
        var request = require('request');
        var fs = require('fs');
        var rapp = this;
        
        fs.readFile( filename, function( err, data) {
            if ( err ) throw err;
            
            // WARNING - I am using a delimiter for both start and end of binary data
            var post = "<IMG>" + data + "</EOF!>";
            
            request.post({
                headers: { 'content-type' : 'text/plain; charset=utf9'},
                url: rapp.cloud_url + "",
                body: post
            },
            function ( error, response, body ) 
            {
                if ( !error && response.statusCode == 200)
                    callback( body );
                else if ( error )
                    console.log (  error );
                else if ( response.statusCode != 200 )
                    console.log ( response.statusCode );
            });
            
        });
    }
}



/// Export
module.exports.RAPPServices = RAPPServices;
