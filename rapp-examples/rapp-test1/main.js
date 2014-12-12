#!/usr/bin/env node

/**
 * This is the main entry point for the HOP.
 * HOP can only execute Javascript or Scheme.
 * Therefore, we rely on Javascript and Node.Js, which will spawn + exec the cpp test1 app.
 * We assume that the cpp app, has already been correctly compiled and linked.
 * 
 * WARNING: You need node.js in a system-wide installation for the below to run.
 * 
 * NOTE: This file, will be generated automatically be the RAPP Submission Service.
 *       Its intended use, is to create the Entry point for the RAPP `test1`
 */

// We need the child process from node.js
var exec = require('child_process').exec;


// DEMO #1: Execute as a seperate process the C++ RAPP
exec ( './test1',
       function ( error, stdout, stderr )
{
    // We may capture the stdout and stderr, and log it for a variety of reasons, such as crashes
    console.log('stdout: ', stdout );
    console.log('stderr: ', stderr );

    // If we capture some kind of fatal error, we could report it to the cloud
    if (error !== null) 
        console.log('exec error: ', error);
    
    // Also, if for any reason, we want to monitor the execution time of this RAPP,
    // we may do so here, by observing the behaviour of the test1 app.
});

// DEMO #2: Execute as a seperate process the Javascript RAPP
exec ( './test1.js',
       function ( error, stdout, stderr )
{
    console.log('stdout: ', stdout );
    console.log('stderr: ', stderr );

    if (error !== null) 
        console.log('exec error: ', error);
});

/** 
 * NOTE: Only one will be executed in real-life scenarios. The above example serves to demonstrate
 *       how easy it is to spawn+exect other apps using node.js
 */